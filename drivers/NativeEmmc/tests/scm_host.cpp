// Tests the actual transport (GuampScm.inc) with a simulated secure monitor; no hardware calls. From
// wifi/driver-modem/tests/scm_host.cpp, updated for the resume with all returned registers and the discovery encoding
// of the guamp kernel's scm.c.
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
  void answer(int code,uint64_t value=0,uint64_t session=0,uint64_t x2=0,uint64_t x5=0) {
    scm::Registers r;r.x[0]=static_cast<uint64_t>(static_cast<int64_t>(code));r.x[1]=value;r.x[2]=x2;r.x[5]=x5;
    r.x[6]=session;r.x[7]=0xdead;answers.push_back(r);
  }
};
int main() {
  {Monitor m;m.answer(1,0x11,0x1234,0x22,0x55);m.answer(1,0x12,0x5678,0x23,0x56);m.answer(0);
   auto r=m.transport().initImage(4,0x80001000);
   assert(!r.transport && !r.value && r.calls==3 && r.resumes==2);
   assert(m.requests[0].x[0]==0x42000201 && m.requests[0].x[1]==0x82);
   assert(m.requests[0].x[2]==4 && m.requests[0].x[3]==0x80001000 && m.requests[0].x[6]==0);
   for(unsigned i=1;i<3;++i)   // resume = every register x0..x6 the monitor returned (scm.c do/while)
     for(unsigned k=0;k<7;++k)assert(m.requests[i].x[k]==m.answers[i-1].x[k]);
   assert(m.requests[1].x[0]==1 && m.requests[1].x[6]==0x1234 && m.requests[2].x[6]==0x5678 && m.requests[2].x[5]==0x56);
   assert(m.requests[1].x[7]==0);}
  {Monitor m;m.answer(-12);m.answer(1,0,0xbeef);m.answer(-12);m.answer(0,0x77);
   auto r=m.transport().memory(1,0x52200000,0x1c00000);
   assert(!r.transport && r.value==0x77 && r.busy==2 && m.sleeps==2);
   assert(m.requests[2].x[0]==1 && m.requests[2].x[6]==0xbeef);
   // busy: fresh call from the original function and arguments with x6 = 0
   assert(m.requests[3].x[0]==0x42000202 && m.requests[3].x[1]==3 && m.requests[3].x[2]==1);
   assert(m.requests[3].x[3]==0x52200000 && m.requests[3].x[4]==0x1c00000 && m.requests[3].x[6]==0);}
  {Monitor m;for(unsigned i=0;i<=scm::kBusyRetries;++i)m.answer(-12);
   auto r=m.transport().authenticate(1);
   assert(r.transport==scm::kBusyLimit && m.sleeps==scm::kBusyRetries);}
  {Monitor m;for(unsigned i=0;i<=scm::kResumeLimit;++i)m.answer(1,0,i+1);
   auto r=m.transport().shutdown(1);assert(r.transport==scm::kInterruptedLimit);}
  // is_scm_armv8: x0 0xc2000601 then 0x82000601, x2 0x82000601 both times, any nonzero x1
  {Monitor m;m.answer(0,1);auto t=m.transport();assert(t.discover() && t.wide);
   assert(m.requests[0].x[0]==0xc2000601 && m.requests[0].x[1]==1 && m.requests[0].x[2]==0x82000601);}
  {Monitor m;m.answer(0,0);m.answer(0,3);auto t=m.transport();assert(t.discover() && !t.wide);
   assert(m.requests[1].x[0]==0x82000601 && m.requests[1].x[2]==0x82000601);
   auto r=t.initImage(1,0x100000000ULL);assert(r.transport==scm::kInvalid && m.requests.size()==2);}
  {Monitor m;m.answer(-1);m.answer(-1);auto t=m.transport();assert(!t.discover());}
  {Monitor m;m.answer(0,1);auto r=m.transport().supported(1);assert(r.value==1);
   assert(m.requests[0].x[0]==0x42000207 && m.requests[0].x[1]==1 && m.requests[0].x[2]==1);}
  {Monitor m;m.answer(0,1);auto r=m.transport().available(2,5);assert(r.value==1);
   assert(m.requests[0].x[0]==0x42000601 && m.requests[0].x[2]==0x02000205);}
  {Monitor m;m.answer(0,0);auto r=m.transport().authenticate(1);
   assert(m.requests[0].x[0]==0x42000205 && m.requests[0].x[1]==1 && m.requests[0].x[2]==1 && !r.transport);}
  assert(scm::code(0xfffffff4ULL)==-12 && scm::code(~11ULL)==-12);
  puts("PASS SCM: interrupted resume with returned x0-x6, busy cadence/bounds, 32/64 discovery as is_scm_armv8, "
       "PAS encoding, 32-bit address refusal");
}
