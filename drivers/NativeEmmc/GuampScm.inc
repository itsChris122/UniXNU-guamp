// SPDX-License-Identifier: BSD-2-Clause-Patent
// Qualcomm SMCCC transport. Normal authenticated PAS services only; no bypasses.
// Protocol checked against the guamp Linux scm.c and qcom_scm-64.c. The caller
// supplies the instruction and sleep callbacks so the exact logic is host-tested.
namespace scm {
struct Registers { uint64_t x[8] = {}; };
typedef void (*Invoke)(void *, const Registers &, Registers &);
typedef void (*Sleep)(void *, uint32_t);
enum : int { kOk=0, kInvalid=-200, kInterruptedLimit=-201, kBusyLimit=-202, kUnavailable=-203 };
constexpr uint32_t kSip=0x02000000, kSmc64=0x40000000, kFast=0x80000000;
constexpr uint32_t kBusyRetries=67, kResumeLimit=1024; // 30ms retry cadence, as guamp's scm.c
inline int code(uint64_t value) { return static_cast<int32_t>(value); }
inline uint32_t function(unsigned service,unsigned command,bool wide,bool fast=false) {
  return kSip | (wide ? kSmc64 : 0) | (fast ? kFast : 0) | ((service&255u)<<8) | (command&255u);
}
struct Result {
  int transport=0;
  uint64_t value=0;
  uint32_t calls=0,resumes=0,busy=0;
};
struct Transport {
  Invoke invoke=nullptr;
  Sleep sleep=nullptr;
  void *context=nullptr;
  bool wide=true;
  // All PAS commands used here fit the four register arguments. Extended
  // argument buffers are deliberately unsupported rather than mis-encoded.
  Result call(unsigned service,unsigned command,uint32_t arginfo,
              uint64_t a=0,uint64_t b=0,uint64_t c=0,uint64_t d=0,bool fast=false) const {
    Result result;
    if (!invoke || (arginfo&15)>4 || (!wide && ((a|b|c|d)>0xffffffffULL))) {
      result.transport=kInvalid;return result;
    }
    for (uint32_t attempt=0;;++attempt) {
      Registers request,answer;
      request.x[0]=function(service,command,wide,fast);
      request.x[1]=arginfo;request.x[2]=a;request.x[3]=b;request.x[4]=c;request.x[5]=d;
      for (uint32_t resumed=0;;++resumed) {
        invoke(context,request,answer);++result.calls;
        int status=code(answer.x[0]);
        if (status!=1) break;
        if (resumed>=kResumeLimit) {result.transport=kInterruptedLimit;return result;}
        // An interrupted SMC resumes with x0=1 and the firmware's returned
        // x6 session token. Reissuing the original function loses that session.
        request.x[0]=1;request.x[6]=answer.x[6];++result.resumes;
      }
      result.transport=code(answer.x[0]);result.value=answer.x[1];
      if (result.transport!=-12 || fast) return result;
      if (attempt>=kBusyRetries || !sleep) {result.transport=kBusyLimit;return result;}
      ++result.busy;sleep(context,30);
      // Retry from the original function with a fresh x6=0 after busy.
    }
  }
  bool discover() {
    wide=true;
    Result r=call(6,1,1,function(6,1,true),0,0,0,true);
    if (!r.transport && r.value==1) return true;
    wide=false;r=call(6,1,1,function(6,1,false),0,0,0,true);
    return !r.transport && r.value==1;
  }
  Result available(unsigned service,unsigned command) const {
    return call(6,1,1,kSip|((service&255u)<<8)|(command&255u));
  }
  Result supported(unsigned pas) const { return call(2,7,1,pas); }
  Result initImage(unsigned pas,uint64_t metadata) const { return call(2,1,2u|(2u<<6),pas,metadata); }
  Result memory(unsigned pas,uint64_t start,uint64_t bytes) const { return call(2,2,3,pas,start,bytes); }
  Result authenticate(unsigned pas) const { return call(2,5,1,pas); }
  Result shutdown(unsigned pas) const { return call(2,6,1,pas); }
};
} // namespace scm
