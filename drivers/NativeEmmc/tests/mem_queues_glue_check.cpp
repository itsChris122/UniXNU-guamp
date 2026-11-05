// Kernel-target compile check (arm64-apple-ios, -Werror) of the GuampPerf.inc "GuampMem: queues" code: GuampMemQueuesCore.h and
// the sampler exactly as GuampPerf.inc has it, cut out by run-mem-queues-tests.sh, with the IOKit log and snprintf declared.
#include <stdint.h>
#include <stddef.h>
extern "C" void IOLog(const char *, ...) __attribute__((format(printf, 1, 2)));
extern "C" int snprintf(char *, size_t, const char *, ...) __attribute__((format(printf, 3, 4)));
#define PAGE_SIZE 4096u
#include "../GuampMemQueuesCore.h"
#include "mem_queues_extract.inc"
void glue_entry() { guampMemQueuesSample(); }
