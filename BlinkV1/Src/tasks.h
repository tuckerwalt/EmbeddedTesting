#ifndef WT_TASKS_H
#define WT_TASKS_H

#include <stdint.h>

#define WT_SHPR3 (*(uint32_t volatile *)(0xE000ED20)) // system handler priority register 3, see programmers manual
#define WT_ICSR (*(uint32_t volatile *)(0xE000ED04)) // interrupt control and state register, see programmers manual

typedef struct {
  void *sp; // stack pointer
  
} OSTask;

typedef void (*OSTaskHandler)();

void OSInit(void);
void OSSched(void);
void OSPendSV(void);

void OSRun(void);
void OSRunCallback(void);

void OSTask_start(OSTask *task, OSTaskHandler taskHandler, void *stackLoc, uint32_t stackSize);
#endif