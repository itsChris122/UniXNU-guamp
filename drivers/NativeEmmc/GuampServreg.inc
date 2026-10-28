// SPDX-License-Identifier: BSD-2-Clause-Patent
// Minimal SERVREG_LOC responder for the exact protection domains declared by
// this phone's modemr.jsn and modemuw.jsn. No arbitrary domain/path lookup.
#ifndef GUAMP_SERVREG_INC
#define GUAMP_SERVREG_INC
namespace guampwifi {
struct GuampServreg {
  static constexpr uint32_t Service=0x40,Instance=0x101,Port=0x4200;
  static constexpr uint32_t RootInstance=180,WlanInstance=180;
  struct Mapping { const char *service,*domain;uint32_t instance; };
  static constexpr Mapping maps[]={
    {"tms/servreg","msm/modem/root_pd",RootInstance},
    {"tms/pdr_enabled","msm/modem/root_pd",RootInstance},
    {"gps/gps_service","msm/modem/root_pd",RootInstance},
    {"kernel/elf_loader","msm/modem/wlan_pd",WlanInstance},
    {"tms/servreg","msm/modem/wlan_pd",WlanInstance},
    {"wlan/fw","msm/modem/wlan_pd",WlanInstance},
  };
  void *ctx=nullptr;
  int (*send)(void *,uint32_t,uint32_t,uint32_t,const uint8_t *,uint32_t)=nullptr;
  bool ready=false;
  int start(void *opaque,int (*tx)(void *,uint32_t,uint32_t,uint32_t,const uint8_t *,uint32_t)) {
    if(!tx)return Invalid;ctx=opaque;send=tx;ready=true;return Ok;
  }
  void stop(){ready=false;ctx=nullptr;send=nullptr;}
  int receive(uint32_t node,uint32_t port,const uint8_t *p,uint32_t n) {
    if(!ready||!p||n<7||node==0xffffffffu||!port)return Invalid;
    uint8_t flags=p[0];uint16_t txn=le16(p+1),msg=le16(p+3),bytes=le16(p+5);
    if(flags!=0||n!=7u+bytes||msg!=0x21)return Unsupported;
    TlvView req={p+7,bytes};if(req.validate())return Protocol;
    uint32_t nameBytes=0;const uint8_t *nameField=req.find(1,&nameBytes);
    if(!nameField||!nameBytes||nameBytes>64)return Protocol;
    char name[65]={};memcpy(name,nameField,nameBytes);
    uint32_t count=0;for(const auto &m:maps)if(equal(name,m.service))count++;
    uint8_t body[768]={};TlvWriter out={body,sizeof(body)};
    uint8_t result[4]={0};if(!out.add(2,result,sizeof(result)))return Range;
    uint8_t total[2]={};put16(total,uint16_t(count));if(!out.add(0x10,total,sizeof(total)))return Range;
    uint8_t revision[2]={1,0};if(!out.add(0x11,revision,sizeof(revision)))return Range;
    if(count) {
      uint8_t list[600]={};uint32_t at=2;put16(list,uint16_t(count));
      for(const auto &m:maps)if(equal(name,m.service)) {
        uint32_t len=strlen(m.domain);if(len>64||at+1+len+9>sizeof(list))return Range;
        list[at++]=uint8_t(len);memcpy(list+at,m.domain,len);at+=len;
        put32(list+at,m.instance);at+=4;list[at++]=0;put32(list+at,0);at+=4;
      }
      if(!out.add(0x12,list,at))return Range;
    }
    uint8_t packet[800]={};packet[0]=2;put16(packet+1,txn);put16(packet+3,msg);put16(packet+5,uint16_t(out.size));
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
