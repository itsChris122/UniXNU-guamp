/*
 * guamp: HakiOS xnu/files/bsd/dev/hakios.net.c (github.com/oosexgang/hackios-xnu-kernel 8851317,
 * Apple Public Source License 2.0, see drivers/GuampWifi/ios/README.md), the en0 of Wi-Fi mode 2
 * (guamp-wifi-start=2, GUAMP_WIFI_REDWOOD). Changed: built outside the XNU tree into libNativeEmmc.a
 * (scripts/build-arm-driver.py --wifi-redwood), so the lock group is made at run time and not by the
 * startup macros; no virtio link; the registry entries come from NativeEmmc GuampWlanNub.inc; the
 * boot arguments are guamp-net-trace=N and guamp-net-names=1; the log lines begin with "GuampWiFi: net:".
 * The one link is the WCN3990 station, drivers/GuampWifi/integration/guamp_xnu_station.c.
 *
 * en0: a network interface that is Ethernet as far as the rest of the kernel and iOS can tell.
 *
 * iOS expects its interfaces from IONetworkingFamily, which this kernel does not have; the BSD
 * half of the kernel does not need it. The interface is made with the kernel's own interface
 * KPI, the way bsd/net/if_fake.c makes its "feth": family Ethernet, the stock Ethernet framing
 * and demultiplexing, one function that is given a frame to send and one call to hand a frame
 * that has arrived to the stack. What iOS's user space makes of an interface that has no
 * IOKit object behind it is another matter: see userland/src/net_test.m.
 *
 * What carries the frames is a link driver's business; sys/hakios.net.h is all that the two
 * know of each other. The first link is QEMU's virtio network device (hakios.virtio_net.c).
 * There can be two interfaces: en0 for the Wi-Fi (or virtio), en1 for the modem's packets
 * through the IPA (hakios.cell.c); each has its link, its state and its thread.
 */
#include <kern/clock.h>
#include <kern/locks.h>
#include <kern/thread.h>
#include <libkern/libkern.h>
#include <mach/mach_types.h>
#include <net/bpf.h>
#include <net/ethernet.h>
#include <net/if.h>
#include <net/if_ether.h>
#include <net/if_media.h>
#include <net/if_types.h>
#include <net/kpi_interface.h>
#include <netinet/in.h>
#include <pexpert/pexpert.h>
#include <sys/kern_event.h>
#include <sys/kpi_mbuf.h>
#include <sys/proc.h>
#include <sys/hakios.net.h>
#include <sys/sockio.h>
#include <sys/systm.h>

extern void guamp_wlan_publish(const char *interface, const uint8_t *address);  /* NativeEmmc GuampWlanNub.inc */
extern void guamp_sm6115_wifi_network_ready(void);  /* drivers/GuampWifi/integration/guamp_xnu_sm6115_runtime.cpp */

static lck_grp_t *net_lck_grp;          /* guamp_wifi_bsd_network_ready() */
/* (XNU keeps its etherbroadcastaddr for BSD_KERNEL_PRIVATE code; this file is built without BSD_KERNEL_PRIVATE) */
static const uint8_t net_broadcast[ETHER_ADDR_LEN] = { 0xff, 0xff, 0xff, 0xff, 0xff, 0xff };

/* One interface: enN on its link. */
struct net_if {
	unsigned        unit;
	const struct hakios_net_link *link;
	bool            up;             /* the link says frames can pass */
	ifnet_t         ifp;
	lck_mtx_t       tx_lock;        /* one frame at a time to the link */
	/* What has gone through: said once a minute while anything moves. */
	struct {
		uint64_t sent, received, sent_bytes, received_bytes, dropped_out, dropped_in;
	} count;
};
static struct net_if net_ifs[HAKIOS_NET_UNITS];

static inline struct net_if *
net_if_of(ifnet_t ifp)
{
	return ifnet_softc(ifp);
}

/* The interface on unit `unit`, once attached; NULL before. */
static struct net_if *
net_if_unit(unsigned unit)
{
	return unit < HAKIOS_NET_UNITS && net_ifs[unit].ifp != NULL ? &net_ifs[unit] : NULL;
}

/*
 * Boot-arg net_trace=N: a line in the log for each of the first N frames, either way.
 * Boot-arg net_names=1: a line for every name asked of a DNS server and for every TCP connection
 * begun from here -- which is how to see what a system that has just been given a network does
 * with it (up to NAMES_MAX lines).
 */
#define NAMES_MAX       3000
static uint32_t net_trace, net_names;

static void
trace_frame(const char *way, const uint8_t *frame, size_t length, size_t whole)
{
	static uint32_t said, names;
	const uint8_t *end = frame + length, *ip = frame + ETHER_HDR_LEN, *l4;
	uint16_t type;
	bool ports;

	if (length < ETHER_HDR_LEN + 28 || (said >= net_trace && (!net_names || names >= NAMES_MAX))) {
		return;
	}
	type = (uint16_t)(frame[12] << 8 | frame[13]);
	l4 = ip + 4 * (ip[0] & 0xf);
	ports = type == ETHERTYPE_IP && (ip[9] == IPPROTO_TCP || ip[9] == IPPROTO_UDP) && l4 + 4 <= end;
	if (net_names && names < NAMES_MAX && ports && way[0] == 'o') {
		if (ip[9] == IPPROTO_UDP && (l4[2] << 8 | l4[3]) == 53 && l4 + 8 + 12 < end) {
			char name[128];
			const uint8_t *label = l4 + 8 + 12;
			size_t n = 0;

			/* the question: lengths and labels, up to a zero */
			while (label < end && *label != 0 && *label < 64 && label + 1 + *label < end && n + *label + 2 < sizeof(name)) {
				memcpy(name + n, label + 1, *label);
				n += *label;
				name[n++] = '.';
				label += 1 + *label;
			}
			name[n ? n - 1 : 0] = 0;
			names++;
			printf("GuampWiFi: net: asks for the name %s (type %u)\n", name, label + 2 < end ? label[1] << 8 | label[2] : 0);
		} else if (ip[9] == IPPROTO_TCP && l4 + 14 <= end && (l4[13] & 0x12) == 0x02) {
			names++;
			printf("GuampWiFi: net: connects to %u.%u.%u.%u port %u\n", ip[16], ip[17], ip[18], ip[19], l4[2] << 8 | l4[3]);
		}
	}
	if (said >= net_trace) {
		return;
	}
	said++;
	if (type == ETHERTYPE_IP) {
		printf("GuampWiFi: net: %s %zu bytes: IPv4 %s %u.%u.%u.%u:%u -> %u.%u.%u.%u:%u%s\n", way, whole, ip[9] == IPPROTO_TCP ? "TCP" :
		    ip[9] == IPPROTO_UDP ? "UDP" : ip[9] == IPPROTO_ICMP ? "ICMP" : "protocol?", ip[12], ip[13], ip[14], ip[15],
		    ports ? l4[0] << 8 | l4[1] : 0, ip[16], ip[17], ip[18], ip[19], ports ? l4[2] << 8 | l4[3] : 0,
		    ip[9] == IPPROTO_TCP && ports && l4 + 14 <= end ? (l4[13] & 2 ? " SYN" : l4[13] & 1 ? " FIN" : l4[13] & 4 ? " RST" : "") : "");
	} else {
		printf("GuampWiFi: net: %s %zu bytes: %s (type %#x) to %02x:%02x:%02x:%02x:%02x:%02x\n", way, whole, type == ETHERTYPE_ARP ? "ARP" :
		    type == ETHERTYPE_IPV6 ? "IPv6" : "?", type, frame[0], frame[1], frame[2], frame[3], frame[4], frame[5]);
	}
}

/* ... of a frame on its way out: the start of it is enough */
static void
trace_packet(mbuf_t packet, size_t length)
{
	uint8_t start[192];
	size_t have = length < sizeof(start) ? length : sizeof(start);

	if ((net_trace || net_names) && mbuf_copydata(packet, 0, have, start) == 0) {
		trace_frame("out", start, have, length);
	}
}

/* ---- the interface -------------------------------------------------------------------------------- */

/* The stack has a frame to send (a chain of mbufs that starts with the Ethernet header). */
static errno_t
net_output(ifnet_t ifp, mbuf_t packet)
{
	struct net_if *n = net_if_of(ifp);
	size_t length = mbuf_pkthdr_len(packet);
	bool sent = false;

	if (length <= HAKIOS_NET_FRAME_MAX && n->up) {
		bpf_tap_out(ifp, DLT_EN10MB, packet, NULL, 0);
		trace_packet(packet, length);
		lck_mtx_lock(&n->tx_lock);
		sent = n->link->transmit(packet, length);
		lck_mtx_unlock(&n->tx_lock);
	}
	if (sent) {
		n->count.sent++;
		n->count.sent_bytes += length;
		ifnet_stat_increment_out(ifp, 1, (uint32_t)length, 0);
	} else {
		n->count.dropped_out++;
		ifnet_stat_increment_out(ifp, 0, 0, 1);
	}
	mbuf_freem(packet);
	return 0;
}

/*
 * guamp 30/10 (fin10: connected, DHCP and DNS fine, but TLS stops after the server's first flight and 114 MB arrive in
 * 80k frames while no TCP connection gets past 12 KB): each IPv4 frame that arrives is checked the way the stack will
 * check it -- header checksum, IP length against the frame, TCP/UDP checksum -- and compared with the last 256 frames
 * (a frame handed up twice). The counts are said with the minute's line, the first 24 bad frames one by one.
 */
static struct {
	uint64_t ipv4, ipv6, arp, other, short_ip, bad_ip, bad_l4, padded, duplicate, tcp, udp;
} rx_check_count;
static uint32_t rx_recent[256];
static unsigned rx_recent_next, rx_bad_said;

static uint32_t
rx_sum(const uint8_t *p, size_t length, uint32_t sum)
{
	for (; length > 1; p += 2, length -= 2) {
		sum += (uint32_t)(p[0] << 8 | p[1]);
	}
	if (length) {
		sum += (uint32_t)p[0] << 8;
	}
	return sum;
}

static uint16_t
rx_fold(uint32_t sum)
{
	while (sum >> 16) {
		sum = (sum & 0xffff) + (sum >> 16);
	}
	return (uint16_t)~sum;
}

static void
rx_bad(const char *why, const uint8_t *frame, size_t length, unsigned ip_length)
{
	const uint8_t *ip = frame + ETHER_HDR_LEN;

	if (rx_bad_said >= 24) {
		return;
	}
	rx_bad_said++;
	printf("GuampWiFi: net: bad frame (%s): %zu bytes, IP length %u, protocol %u, %u.%u.%u.%u -> %u.%u.%u.%u; "
	    "%02x%02x%02x%02x %02x%02x%02x%02x %02x%02x%02x%02x %02x%02x%02x%02x %02x%02x%02x%02x | %02x%02x%02x%02x %02x%02x%02x%02x
",
	    why, length, ip_length, ip[9], ip[12], ip[13], ip[14], ip[15], ip[16], ip[17], ip[18], ip[19],
	    ip[20], ip[21], ip[22], ip[23], ip[24], ip[25], ip[26], ip[27], ip[28], ip[29], ip[30], ip[31], ip[32], ip[33], ip[34],
	    ip[35], ip[36], ip[37], ip[38], ip[39], frame[length - 8], frame[length - 7], frame[length - 6], frame[length - 5],
	    frame[length - 4], frame[length - 3], frame[length - 2], frame[length - 1]);
}

static void
rx_check(const uint8_t *frame, size_t length)
{
	const uint8_t *ip = frame + ETHER_HDR_LEN;
	uint16_t type;
	uint32_t hash = 2166136261u;

	if (length < ETHER_HDR_LEN) {
		return;
	}
	for (size_t i = 0; i < length && i < 96; i++) {
		hash = (hash ^ frame[i]) * 16777619u;
	}
	hash ^= (uint32_t)length;
	for (unsigned i = 0; i < 256; i++) {
		if (rx_recent[i] == hash) {
			rx_check_count.duplicate++;
			break;
		}
	}
	rx_recent[rx_recent_next++ & 255] = hash;
	type = (uint16_t)(frame[12] << 8 | frame[13]);
	if (type == ETHERTYPE_IPV6) {
		rx_check_count.ipv6++;
		return;
	}
	if (type == ETHERTYPE_ARP) {
		rx_check_count.arp++;
		return;
	}
	if (type != ETHERTYPE_IP || length < ETHER_HDR_LEN + 40) {
		rx_check_count.other++;
		return;
	}
	rx_check_count.ipv4++;
	unsigned header = 4u * (ip[0] & 0xf), ip_length = (unsigned)(ip[2] << 8 | ip[3]);
	if ((ip[0] >> 4) != 4 || header < 20 || ip_length < header || ETHER_HDR_LEN + ip_length > length) {
		rx_check_count.short_ip++;
		rx_bad("IP length", frame, length, ip_length);
		return;
	}
	if (ETHER_HDR_LEN + ip_length < length) {
		rx_check_count.padded++;
	}
	if (rx_fold(rx_sum(ip, header, 0)) != 0) {
		rx_check_count.bad_ip++;
		rx_bad("IP checksum", frame, length, ip_length);
		return;
	}
	if ((ip[6] & 0x3f) || ip[7]) {
		return;         /* a fragment: its checksum covers the whole datagram */
	}
	if (ip[9] != IPPROTO_TCP && ip[9] != IPPROTO_UDP) {
		return;
	}
	const uint8_t *l4 = ip + header;
	unsigned l4_length = ip_length - header;
	if (ip[9] == IPPROTO_TCP) {
		rx_check_count.tcp++;
	} else {
		rx_check_count.udp++;
		if (l4_length < 8 || (l4[6] == 0 && l4[7] == 0)) {
			return;         /* no UDP checksum */
		}
	}
	uint32_t sum = rx_sum(ip + 12, 8, 0) + ip[9] + l4_length;
	if (rx_fold(rx_sum(l4, l4_length, sum)) != 0) {
		rx_check_count.bad_l4++;
		rx_bad(ip[9] == IPPROTO_TCP ? "TCP checksum" : "UDP checksum", frame, length, ip_length);
	}
}

/* A frame has arrived on unit `unit`. */
void
hakios_net_input_unit(unsigned unit, const uint8_t *frame, size_t length)
{
	struct net_if *n = net_if_unit(unit);
	struct ifnet_stat_increment_param stats = { .packets_in = 1, .bytes_in = (uint32_t)length };
	unsigned int chunks = 1;
	mbuf_t packet = NULL;
	char *header;

	if (n == NULL) {
		return;
	}
	trace_frame("in ", frame, length, length);
	if (n->link->wireless) {
		rx_check(frame, length);
	}
	if (length < ETHER_HDR_LEN || length > HAKIOS_NET_FRAME_MAX || mbuf_allocpacket(MBUF_DONTWAIT, length, &chunks, &packet) != 0) {
		n->count.dropped_in++;
		return;
	}
	if (mbuf_copyback(packet, 0, length, frame, MBUF_DONTWAIT) != 0) {
		mbuf_freem(packet);
		n->count.dropped_in++;
		return;
	}
	/* as every Ethernet driver hands a frame up: the header noted and stepped over */
	header = mbuf_data(packet);
	mbuf_pkthdr_setrcvif(packet, n->ifp);
	mbuf_pkthdr_setheader(packet, header);
	mbuf_pkthdr_adjustlen(packet, -ETHER_HDR_LEN);
	mbuf_setdata(packet, header + ETHER_HDR_LEN, length - ETHER_HDR_LEN);
	bpf_tap_in(n->ifp, DLT_EN10MB, packet, header, ETHER_HDR_LEN);
	n->count.received++;
	n->count.received_bytes += length;
	ifnet_input(n->ifp, packet, &stats);
}

void
hakios_net_input(const uint8_t *frame, size_t length)
{
	hakios_net_input_unit(0, frame, length);
}

static unsigned net_ioctls_said;

static errno_t
net_ioctl(ifnet_t ifp, u_long command, void *data)
{
	struct net_if *n = net_if_of(ifp);
	/* (a radio says it is one here too: IO80211.framework lists the interfaces whose media is 802.11) */
	const int media[] = { (n->link->wireless ? IFM_IEEE80211 : IFM_ETHER) | IFM_AUTO };
	struct ifreq *request = data;
	struct ifmediareq *media_request = data;
	user_addr_t list;
	errno_t error = 0;

	if (n->link->wireless && net_ioctls_said < 80) {
		char who[32] = "?";

		/* (while iOS's Wi-Fi side is being got to talk to this: who asks the interface what) */
		proc_selfname(who, sizeof(who));
		if (strcmp(who, "configd") != 0) {
			net_ioctls_said++;
			printf("GuampWiFi: net: en0 is asked %#lx by %s\n", command, who);
		}
	}
	switch (command) {
	case SIOCSIFADDR:
		ifnet_set_flags(ifp, IFF_UP, IFF_UP);
		break;
	case SIOCSIFFLAGS:
		/* running whenever it is up: the link is there or it is not, nothing is switched on */
		ifnet_set_flags(ifp, (ifnet_flags(ifp) & IFF_UP) ? IFF_RUNNING : 0, IFF_RUNNING);
		break;
	case SIOCGIFMEDIA32:
	case SIOCGIFMEDIA64:
		/* what configd asks to learn whether the cable is in */
		list = command == SIOCGIFMEDIA64 ? ((struct ifmediareq64 *)media_request)->ifmu_ulist :
		    CAST_USER_ADDR_T(((struct ifmediareq32 *)media_request)->ifmu_ulist);
		media_request->ifm_active = media_request->ifm_current = media[0];
		media_request->ifm_mask = 0;
		media_request->ifm_status = IFM_AVALID | (n->up ? IFM_ACTIVE : 0);
		if (list != USER_ADDR_NULL && media_request->ifm_count > 0) {
			error = copyout(media, list, sizeof(media));
		}
		media_request->ifm_count = 1;
		break;
	case SIOCGIFDEVMTU:
		request->ifr_devmtu.ifdm_current = ifnet_mtu(ifp);
		request->ifr_devmtu.ifdm_max = ETHERMTU;
		request->ifr_devmtu.ifdm_min = IF_MINMTU;
		break;
	case SIOCSIFMTU:
		error = request->ifr_mtu > ETHERMTU || request->ifr_mtu < IF_MINMTU ? EINVAL : ifnet_set_mtu(ifp, request->ifr_mtu);
		break;
	case SIOCADDMULTI:
	case SIOCDELMULTI:
		break;                          /* the link hands us everything anyway */
	default:
		/* what iOS's Wi-Fi side asks of a radio's interface: bsd/dev/hakios.apple80211.c */
		error = n->link->wireless ? hakios_apple80211_ioctl(command, data) : EOPNOTSUPP;
		break;
	}
	return error;
}

/* Tell whoever listens for kernel events (configd does) that the link is up, or down. */
static void
net_link_event(ifnet_t ifp, uint32_t code)
{
	_Alignas(struct kern_event_msg) char message[sizeof(struct kern_event_msg) + sizeof(struct net_event_data)] = { 0 };
	struct kern_event_msg *header = (struct kern_event_msg *)message;
	struct net_event_data *about = (struct net_event_data *)(header + 1);

	header->total_size = sizeof(message);
	header->vendor_code = KEV_VENDOR_APPLE;
	header->kev_class = KEV_NETWORK_CLASS;
	header->kev_subclass = KEV_DL_SUBCLASS;
	header->event_code = code;
	about->if_family = ifnet_family(ifp);
	about->if_unit = ifnet_unit(ifp);
	strlcpy(about->if_name, ifnet_name(ifp), IFNAMSIZ);
	ifnet_event(ifp, header);
}

void
hakios_net_link_state_unit(unsigned unit, bool up)
{
	struct net_if *n = net_if_unit(unit);

	if (n == NULL || up == n->up) {
		return;
	}
	n->up = up;
	printf("GuampWiFi: net: en%u's link (%s) is %s\n", n->unit, n->link->name, up ? "up" : "down");
	net_link_event(n->ifp, up ? KEV_DL_LINK_ON : KEV_DL_LINK_OFF);
}

void
hakios_net_link_state(bool up)
{
	hakios_net_link_state_unit(0, up);
}

/* The interface's own thread: for a link that wants to be asked, and for the minute's line in the log. */
static void
net_thread(void *parameter, __unused wait_result_t result)
{
	struct net_if *n = parameter;
	uint64_t said = 0, total;
	unsigned quiet = 0;

	for (unsigned tick = 1;; tick++) {
		quiet = n->link->poll != NULL && n->link->poll() ? 0 : quiet + 1;
		/* about once a minute, if anything has moved since */
		total = n->count.sent + n->count.received + n->count.dropped_out + n->count.dropped_in;
		if (tick % 8192 == 0 && total != said) {
			said = total;
			printf("GuampWiFi: net: en%u so far %llu frames sent (%llu bytes), %llu received (%llu bytes); dropped: %llu out, %llu in\n",
			    n->unit, n->count.sent, n->count.sent_bytes, n->count.received, n->count.received_bytes,
			    n->count.dropped_out, n->count.dropped_in);
			if (n->link->wireless) {
				printf("GuampWiFi: net: en%u received IPv4 %llu (TCP %llu, UDP %llu), IPv6 %llu, ARP %llu, other %llu; bad: IP length %llu, "
				    "IP checksum %llu, TCP/UDP checksum %llu; padded %llu, same as a recent frame %llu
", n->unit, rx_check_count.ipv4,
				    rx_check_count.tcp, rx_check_count.udp, rx_check_count.ipv6, rx_check_count.arp, rx_check_count.other,
				    rx_check_count.short_ip, rx_check_count.bad_ip, rx_check_count.bad_l4, rx_check_count.padded, rx_check_count.duplicate);
			}
		}
		delay_for_interval(quiet < 100 ? 2 : 10, NSEC_PER_MSEC);
	}
}

/*
 * guamp-wifi-ui (29/10, analisi/wifi/IMPOSTAZIONI-WIFI-20261029.md): en0 from bsd_init on, before the radio is up.
 * Settings lists Wi-Fi from MobileGestalt (ADT /chosen/mac-address-wifi0), but wifid has no device until an 802.11
 * interface exists, so the row cannot be used. The placeholder link carries nothing (its link is down, so net_output()
 * never calls transmit) and the station side is absent (guamp_sta.c: power reads off, a power-on is remembered).
 * When the station attaches its link takes the placeholder's place on the same en0 (hakios_net_attach below).
 */
extern bool guamp_wlan_adt_address(uint8_t *address);              /* NativeEmmc GuampWlanNub.inc */
extern void guamp_wlan_set_address(const uint8_t *address);        /* likewise */

static bool
placeholder_transmit(__unused mbuf_t packet, __unused size_t length)
{
	return false;
}

static struct hakios_net_link net_placeholder = {
	.name = "guamp Wi-Fi (radio not up yet)",
	.unit = 0,
	.wireless = true,
	.up = false,
	.transmit = placeholder_transmit,
};

/* The station's link on the placeholder's en0: same interface, same services, the radio's address. */
static bool
net_take_over(struct net_if *n, const struct hakios_net_link *link)
{
	lck_mtx_lock(&n->tx_lock);
	n->link = link;
	lck_mtx_unlock(&n->tx_lock);
	if (bcmp(link->address, net_placeholder.address, ETHER_ADDR_LEN) != 0) {
		ifnet_set_lladdr(n->ifp, link->address, ETHER_ADDR_LEN);
		guamp_wlan_set_address(link->address);
	}
	printf("GuampWiFi: net: en%u taken over by %s (was the placeholder), link %s\n", n->unit, link->name, link->up ? "up" : "down");
	hakios_net_link_state_unit(n->unit, link->up);
	return true;
}

bool
hakios_net_attach(const struct hakios_net_link *link)
{
	unsigned unit = link->unit;
	struct net_if *n = unit < HAKIOS_NET_UNITS ? &net_ifs[unit] : NULL;
	char uniqueid[16];
	struct ifnet_init_eparams init = {
		.ver = IFNET_INIT_CURRENT_VERSION,
		.len = sizeof(init),
		/*
		 * The simple model: every frame straight to net_output(). And no Skywalk "nexus" made for
		 * the interface by itself: sockets are all that is asked of it so far.
		 */
		.flags = IFNET_INIT_LEGACY | IFNET_INIT_NX_NOAUTO,
		.uniqueid = uniqueid,
		.uniqueid_len = 10,
		.name = "en",
		.unit = unit,
		.softc = n,
		.family = IFNET_FAMILY_ETHERNET,
		.subfamily = link->wireless ? IFNET_SUBFAMILY_WIFI : IFNET_SUBFAMILY_ANY,
		.type = IFT_ETHER,
		.output = net_output,
		.demux = ether_demux,
		.add_proto = ether_add_proto,
		.del_proto = ether_del_proto,
		.check_multi = ether_check_multi,
		.framer_extended = ether_frameout_extended,
		.ioctl = net_ioctl,
		.broadcast_addr = net_broadcast,
		.broadcast_len = ETHER_ADDR_LEN,
	};
	const uint8_t *address = link->address;
	ifnet_t ifp = NULL;
	thread_t thread;
	errno_t error;

	if (n == NULL) {
		printf("GuampWiFi: net: there is no unit %u for %s\n", unit, link->name);
		return false;
	}
	if (net_lck_grp == NULL) {
		printf("GuampWiFi: net: BSD networking is not up yet (no bsd_init hook); %s is not attached\n", link->name);
		return false;
	}
	if (n->link == &net_placeholder && link != &net_placeholder && link->wireless) {
		return net_take_over(n, link);
	}
	if (n->link != NULL) {
		printf("GuampWiFi: net: en%u is on %s already; %s is not attached\n", unit, n->link->name, link->name);
		return false;
	}
	snprintf(uniqueid, sizeof(uniqueid), "guamp-en%u", unit);
	n->unit = unit;
	n->link = link;
	n->up = link->up;
	lck_mtx_init(&n->tx_lock, net_lck_grp, LCK_ATTR_NULL);
	error = ifnet_allocate_extended(&init, &ifp);
	if (error != 0) {
		printf("GuampWiFi: net: the interface en%u cannot be made: error %d\n", unit, error);
		n->link = NULL;
		return false;
	}
	ifnet_set_capabilities_enabled(ifp, 0, -1);
	ifnet_set_addrlen(ifp, ETHER_ADDR_LEN);
	ifnet_set_baudrate(ifp, 100 * 1000 * 1000);
	ifnet_set_mtu(ifp, ETHERMTU);
	ifnet_set_flags(ifp, IFF_BROADCAST | IFF_MULTICAST | IFF_SIMPLEX, 0xffff);
	ifnet_set_hdrlen(ifp, ETHER_HDR_LEN);
	ifnet_set_offload(ifp, 0);
	error = ifnet_attach(ifp, NULL);
	if (error != 0) {
		printf("GuampWiFi: net: the interface en%u cannot be attached: error %d\n", unit, error);
		ifnet_release(ifp);
		n->link = NULL;
		return false;
	}
	ifnet_set_lladdr(ifp, address, ETHER_ADDR_LEN);
	bpfattach(ifp, DLT_EN10MB, ETHER_HDR_LEN);
	n->ifp = ifp;
	if (n->up) {
		net_link_event(ifp, KEV_DL_LINK_ON);
	}
	if (link->wireless) {
		/* (a radio's address is the phone's own: not for the log; and iOS's Wi-Fi side finds the interface by a service) */
		printf("GuampWiFi: net: en%u attached on %s (a radio), link %s\n", unit, link->name, n->up ? "up" : "down");
		guamp_wlan_publish(uniqueid + 6, address);
	} else {
		printf("GuampWiFi: net: en%u attached on %s, address %02x:%02x:%02x:%02x:%02x:%02x, link %s\n", unit, link->name, address[0],
		    address[1], address[2], address[3], address[4], address[5], n->up ? "up" : "down");
	}
	if (kernel_thread_start(net_thread, n, &thread) == KERN_SUCCESS) {
		thread_deallocate(thread);
	}
	return true;
}

/* bsd_init(), once, right after net_init_run(): see sys/hakios.net.h */
void
guamp_wifi_bsd_network_ready(void)
{
	unsigned mode = 0, ui;

	/*
	 * Mode 2 (the iOS station) uses all of this. guamp-wifi-ui (default 1 with mode 2, 0 otherwise; =0 turns it off)
	 * attaches the placeholder en0 so that Settings has a Wi-Fi device (switch off) before the radio is up, or without
	 * a radio at all (guamp-wifi-ui=1 alone). Mode 1 and no mode, without guamp-wifi-ui=1: nothing at all is done.
	 */
	PE_parse_boot_argn("guamp-wifi-start", &mode, sizeof(mode));
	ui = mode == 2;
	PE_parse_boot_argn("guamp-wifi-ui", &ui, sizeof(ui));
	if ((mode != 2 && ui != 1) || net_lck_grp != NULL) {
		return;
	}
	net_lck_grp = lck_grp_alloc_init("guamp-net", LCK_GRP_ATTR_NULL);
	guamp_sta_init();
	guamp_apple80211_init();
	PE_parse_boot_argn("guamp-net-trace", &net_trace, sizeof(net_trace));
	PE_parse_boot_argn("guamp-net-names", &net_names, sizeof(net_names));
	if (ui) {
		if (!guamp_wlan_adt_address(net_placeholder.address)) {
			/* (no /chosen/mac-address-wifi0: a locally administered address, 02:47:55 as the Bluetooth one) */
			static const uint8_t laa[ETHER_ADDR_LEN] = { 0x02, 0x47, 0x55, 0x57, 0x4c, 0x30 };
			memcpy(net_placeholder.address, laa, ETHER_ADDR_LEN);
		}
		printf("GuampWiFi: net: guamp-wifi-ui: placeholder en0 for Settings (%s)\n",
		    hakios_net_attach(&net_placeholder) ? "attached, switch off until the radio is up" : "NOT attached");
	}
	if (mode != 2) {
		printf("GuampWiFi: net: BSD networking is up; no radio in this run (guamp-wifi-start is not 2)\n");
		return;
	}
	printf("GuampWiFi: net: BSD networking is up; the iOS station may attach en0\n");
	guamp_sm6115_wifi_network_ready();
}
