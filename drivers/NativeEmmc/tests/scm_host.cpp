// Tests the actual transport with a simulated secure monitor; no hardware calls.
#include <stdint.h>
#include <assert.h>
#include <vector>
#include <stdio.h>
#include "../GuampScm.inc"
struct Monitor {
  std::vector<scm::Registers> answers,requests;
  unsigned sleeps=0;
  static void invoke(void *ctx,const scm::Registers &in,scm::Registers &out) {
    auto &m=*static_cast<Monitor *>(ctx);m.requests.push_back(in);
    assert(m.requests.size()<=m.answers.size());out=m.answers[m.requests.size()-1];
  }
  static void sleep(void *ctx,uint32_t ms) {assert(ms==30);++static_cast<Monitor *>(ctx)->sleeps;}
  scm::Transport transport() {return {invoke,sleep,this,true};}
  void answer(int code,uint64_t value=0,uint64_t session=0) {
    scm::Registers r;r.x[0]=static_cast<uint64_t>(static_cast<int64_t>(code));r.x[1]=value;r.x[6]=session;answers.push_back(r);
  }
};
int main() {
  {Monitor m;m.answer(1,0,0x1234);m.answer(1,0,0x5678);m.answer(0);
   auto r=m.transport().initImage(4,0x80001000);
   assert(!r.transport && !r.value && r.calls==3 && r.resumes==2);
   assert(m.requests[0].x[0]==0x42000201 && m.requests[0].x[1]==0x82);
   assert(m.requests[1].x[0]==1 && m.requests[1].x[6]==0x1234);
   assert(m.requests[2].x[0]==1 && m.requests[2].x[6]==0x5678);
   assert(m.requests[2].x[2]==4 && m.requests[2].x[3]==0x80001000);}
  {Monitor m;m.answer(-12);m.answer(1,0,0xbeef);m.answer(-12);m.answer(0,0x77);
   auto r=m.transport().memory(4,0x4ab00000,0x6900000);
   assert(!r.transport && r.value==0x77 && r.busy==2 && m.sleeps==2);
   assert(m.requests[2].x[0]==1 && m.requests[2].x[6]==0xbeef);
   assert(m.requests[3].x[0]==0x42000202 && m.requests[3].x[6]==0);}
  {Monitor m;for(unsigned i=0;i<=scm::kBusyRetries;++i)m.answer(-12);
   auto r=m.transport().authenticate(4);
   assert(r.transport==scm::kBusyLimit && m.sleeps==scm::kBusyRetries);}
  {Monitor m;for(unsigned i=0;i<=scm::kResumeLimit;++i)m.answer(1,0,i+1);
   auto r=m.transport().shutdown(4);assert(r.transport==scm::kInterruptedLimit);}
  {Monitor m;m.answer(0,1);auto t=m.transport();assert(t.discover() && t.wide);
   assert(m.requests[0].x[0]==0xc2000601 && m.requests[0].x[2]==0x42000601);}
  {Monitor m;m.answer(-1);m.answer(0,1);auto t=m.transport();assert(t.discover() && !t.wide);
   assert(m.requests[1].x[0]==0x82000601 && m.requests[1].x[2]==0x02000601);
   auto r=t.initImage(4,0x100000000ULL);assert(r.transport==scm::kInvalid && m.requests.size()==2);}
  {Monitor m;m.answer(0,1);auto r=m.transport().supported(4);assert(r.value==1);
   assert(m.requests[0].x[0]==0x42000207 && m.requests[0].x[2]==4);}
  {Monitor m;m.answer(0,1);auto r=m.transport().available(2,5);assert(r.value==1);
   assert(m.requests[0].x[2]==0x02000205);}
  assert(scm::code(0xfffffff4ULL)==-12 && scm::code(~11ULL)==-12);
  puts("PASS SCM: interrupted x0/x6 resumes, busy cadence/bounds,32/64 discovery, PAS encoding,32-bit address refusal");
}
