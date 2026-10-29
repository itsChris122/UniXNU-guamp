// SPDX-License-Identifier: BSD-2-Clause-Patent
// SERVREG_LOC (service 0x40, v1 i1) responder, as linux-msm/pd-mapper: GET_DOMAIN_LIST (0x21) answers from the
// protection-domain database of this phone, i.e. every *.jsn of the modem firmware partition (/image): modemr.jsn
// (msm/modem/root_pd 180), modemuw.jsn (msm/modem/wlan_pd 180), adspr/adsps/adspua.jsn (msm/adsp/root_pd,
// sensor_pd, audio_pd 74), cdspr.jsn (msm/cdsp/root_pd 76). Encoding as qmi_servreg_loc_get_domain_list_resp
// (service-locator-private.h): TLV 2 result, 0x10 total_domains u16, 0x11 db_rev_count u16, 0x12 domain_list =
// u8 count + entries {u8 name length, name, u32 instance, u8 service_data_valid, u32 service_data}; the request's
// optional 0x10 domain_offset pages the list (service-locator.c reads until total_domains).
#ifndef GUAMP_SERVREG_INC
#define GUAMP_SERVREG_INC
namespace guampwifi {
struct GuampServreg {
  static constexpr uint32_t Service=0x40,Instance=0x101,Port=0x4200;
  static constexpr uint32_t RootInstance=180,WlanInstance=180,AdspInstance=74,CdspInstance=76,MaxList=32;
  struct Mapping { const char *service,*domain;uint32_t instance; };
  static constexpr Mapping maps[]={
    {"tms/servreg","msm/modem/root_pd",RootInstance},
    {"tms/pdr_enabled","msm/modem/root_pd",RootInstance},
    {"gps/gps_service","msm/modem/root_pd",RootInstance},
    {"kernel/elf_loader","msm/modem/wlan_pd",WlanInstance},
    {"tms/servreg","msm/modem/wlan_pd",WlanInstance},
    {"wlan/fw","msm/modem/wlan_pd",WlanInstance},
    {"tms/servreg","msm/adsp/root_pd",AdspInstance},
    {"tms/servreg","msm/adsp/sensor_pd",AdspInstance},
    {"tms/servreg","msm/adsp/audio_pd",AdspInstance},
    {"avs/audio","msm/adsp/audio_pd",AdspInstance},
    {"tms/servreg","msm/cdsp/root_pd",CdspInstance},
  };
  void *ctx=nullptr;
  int (*send)(void *,uint32_t,uint32_t,uint32_t,const uint8_t *,uint32_t)=nullptr;
  void *traceCtx=nullptr;
  void (*trace)(void *,const char *what,const char *name,int e,uint64_t a,uint64_t b,uint64_t c)=nullptr;
  uint32_t requests=0;
  bool ready=false;
  int start(void *opaque,int (*tx)(void *,uint32_t,uint32_t,uint32_t,const uint8_t *,uint32_t)) {
    if(!tx)return Invalid;ctx=opaque;send=tx;ready=true;requests=0;return Ok;
  }
  void stop(){ready=false;ctx=nullptr;send=nullptr;}
  int receive(uint32_t node,uint32_t port,const uint8_t *p,uint32_t n) {
    if(!ready||!p||n<7||node==0xffffffffu||!port)return Invalid;
    uint8_t flags=p[0];uint16_t txn=le16(p+1),msg=le16(p+3),bytes=le16(p+5);
    if(flags!=0||n!=7u+bytes) return Unsupported;
    if(msg!=0x21) {
      if(trace) trace(traceCtx,"unsupported message: id/node/port",nullptr,Unsupported,msg,node,port);
      return Unsupported;
    }
    TlvView req={p+7,bytes};if(req.validate())return Protocol;
    uint32_t nameBytes=0;const uint8_t *nameField=req.find(1,&nameBytes);
    if(!nameField||!nameBytes||nameBytes>64)return Protocol;
    char name[65]={};memcpy(name,nameField,nameBytes);
    uint32_t offset=0,offBytes=0;const uint8_t *off=req.find(0x10,&offBytes);
    if(off&&offBytes==4) offset=le32(off);
    uint32_t total=0;for(const auto &m:maps)if(equal(name,m.service))total++;
    uint8_t body[1024]={};TlvWriter out={body,sizeof(body)};
    uint8_t result[4]={0};if(!out.add(2,result,sizeof(result)))return Range;
    uint8_t t[2]={};put16(t,uint16_t(total));if(!out.add(0x10,t,sizeof(t)))return Range;
    uint8_t revision[2]={1,0};if(!out.add(0x11,revision,sizeof(revision)))return Range;
    uint32_t listed=0;
    if(offset<total) {
      uint8_t list[900]={};uint32_t at=1,k=0;
      for(const auto &m:maps) {
        if(!equal(name,m.service)) continue;
        if(k++<offset) continue;
        uint32_t len=uint32_t(strlen(m.domain));
        if(listed==MaxList||at+1+len+9>sizeof(list)) break;
        list[at++]=uint8_t(len);memcpy(list+at,m.domain,len);at+=len;
        put32(list+at,m.instance);at+=4;list[at++]=0;put32(list+at,0);at+=4;
        listed++;
      }
      list[0]=uint8_t(listed);   // QMI_DATA_LEN of domain_list: one byte (elem_size sizeof(uint8_t))
      if(!out.add(0x12,list,at))return Range;
    }
    requests++;
    if(trace) trace(traceCtx,"GET_DOMAIN_LIST -> entries/total/offset",name,Ok,listed,total,offset);
    uint8_t packet[1040]={};packet[0]=2;put16(packet+1,txn);put16(packet+3,msg);put16(packet+5,uint16_t(out.size));
    memcpy(packet+7,out.data,out.size);
    return send(ctx,Port,node,port,packet,7+out.size);
  }
private:
  static bool equal(const char *a,const char *b) {
    if(!a||!b)return false;while(*a&&*b)if(*a++!=*b++)return false;return *a==*b;
  }
};
} // namespace guampwifi
#endif
