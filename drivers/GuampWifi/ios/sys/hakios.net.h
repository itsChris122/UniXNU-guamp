/*
 * guamp: HakiOS xnu/files/bsd/sys/hakios.net.h (github.com/oosexgang/hackios-xnu-kernel 8851317,
 * Apple Public Source License 2.0, see drivers/GuampWifi/ios/README.md), kept as the station/net
 * boundary that drivers/GuampWifi/integration/guamp_xnu_station.c is written against. Changed:
 * no virtio link; hakios_net_init() became guamp_wifi_bsd_network_ready() (the bsd_init hook).
 * The implementation here is drivers/GuampWifi/ios/guamp_net.c.
 *
 * en0, and what stands behind it.
 *
 * The network interface iOS uses here is made in bsd/dev/hakios.net.c with the kernel's own
 * interface KPI: Ethernet as far as the rest of the kernel and user space can tell. What carries
 * the frames is a "link", a driver of its own that knows nothing about interfaces: QEMU's virtio
 * network device today (bsd/dev/hakios.virtio_net.c), the phone's radio or frames carried over
 * the boot loader's USB channel tomorrow. This is all the two know of each other.
 *
 * A link driver
 *   - fills in a struct hakios_net_link and calls hakios_net_attach() once, when it is ready
 *     to carry frames (for a radio: when its firmware is up, not when it has joined a network);
 *   - hands every frame that arrives to hakios_net_input(): from its own thread, or from
 *     the poll function the interface's thread calls for it. Not from interrupt context;
 *   - says with hakios_net_link_state() when frames can pass and when they no longer can
 *     (the cable, or for a radio: associated and the keys set). configd starts DHCP on that.
 * A frame is what goes over an Ethernet cable without the checksum at its end: destination,
 * source, type, payload; at most HAKIOS_NET_FRAME_MAX bytes. A radio whose frames are 802.11
 * converts in its driver (or has its firmware do it: "Ethernet decap/encap").
 */
#ifndef _SYS_HAKIOS_NET_H_
#define _SYS_HAKIOS_NET_H_

#ifdef KERNEL
#include <sys/kpi_mbuf.h>

#define HAKIOS_NET_FRAME_MAX   1514
#define HAKIOS_NET_UNITS       2       /* en0: the radio (or virtio); en1: the modem's packets (hakios.cell.c) */

struct hakios_net_link {
	const char *    name;           /* for the log */
	unsigned        unit;           /* which enN this is (0 unless said otherwise) */
	uint8_t         address[6];     /* the interface's Ethernet address */
	bool            wireless;       /* the interface is told to be Wi-Fi (its "subfamily"), which is how the
	                                 * network stack and configd tell a radio from a cable */
	bool            up;             /* frames can pass from the start (else: wait for hakios_net_link_state()) */
	/*
	 * Send one frame: a chain of mbufs, `length` bytes in all, the Ethernet header first. The
	 * packet stays the caller's: copy it (mbuf_copydata()). Returns false if it had to be dropped.
	 * Called by one thread at a time, never from interrupt context.
	 */
	bool            (*transmit)(mbuf_t packet, size_t length);
	/*
	 * Optional. Look whether frames have arrived and call hakios_net_input() for each; returns
	 * how many there were. The interface's thread calls it every 2 ms while frames are coming and
	 * every 10 ms when none has for a while. A link that has a thread or an interrupt of its own
	 * leaves it NULL.
	 */
	unsigned        (*poll)(void);
};

/* Make en<unit> on top of that link (the structure must stay). One link per unit, once: false if there is one already. */
bool hakios_net_attach(const struct hakios_net_link *link);
/* A frame has arrived (on en0 / on en<unit>). Copied; the buffer is the caller's again on return. */
void hakios_net_input(const uint8_t *frame, size_t length);
void hakios_net_input_unit(unsigned unit, const uint8_t *frame, size_t length);
/* Frames can pass now / can no longer pass (on en0 / on en<unit>). */
void hakios_net_link_state(bool up);
void hakios_net_link_state_unit(unsigned unit, bool up);

/*
 * guamp: called once by bsd_init() right after net_init_run() (patch zzzzzzzz-guamp-wifi-network-ready);
 * with guamp-wifi-start=2 only (else it does nothing) it makes the locks of the three files, reads
 * guamp-net-trace/guamp-net-names and tells the Wi-Fi runtime that BSD networking is up (guamp_sm6115_wifi_network_ready()).
 */
void guamp_wifi_bsd_network_ready(void);
void guamp_sta_init(void);              /* guamp_sta.c, from guamp_wifi_bsd_network_ready() */
void guamp_apple80211_init(void);       /* guamp_apple80211.c, likewise */
/* an Apple80211 request (SIOCGA80211, SIOCSA80211) for the interface of a radio: guamp_apple80211.c */
errno_t hakios_apple80211_ioctl(u_long command, void *data);
#endif /* KERNEL */

#endif /* _SYS_HAKIOS_NET_H_ */
