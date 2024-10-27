#include <stdint.h>
#include "tasks.h"
#include "stm32c0xx_hal.h"

OSTask * volatile OSCurrent;
OSTask * volatile OSNext;

void OSInit(void)
{
  // set SYSTICK priority to second last (note: done in bspinit)
  //SHPR3 &= ~(0x3U << 30);
  //SHPR3 |= (0x2U << 30);
  //set PENDSV priority to the lowest value
  WT_SHPR3 |= (0x3U << 22);
}

void OSSched(void)
{
  extern OSTask led4task;
  extern OSTask whiteledtask;
  
  if (OSCurrent == &led4task)
  {
    OSNext = &whiteledtask;
  }
  else
  {
    OSNext = &led4task;
  }
  
  if (OSNext != OSCurrent)
  {
    WT_ICSR |= (0x1U << 28); // set PENDSVSET interrupt pending
  }
}

void OSTask_start(OSTask *task, OSTaskHandler taskHandler, void *stackLoc, uint32_t stackSize)
{
  // Set the stack pointer to the end of stack (align on an 8-byte boundary)
  uint32_t *sp = (uint32_t *)(((uintptr_t)stackLoc + stackSize) & ~0x7);
  uint32_t *stack_end;
  
  *(--sp) = (0x1U << 24); // Thumb state in xPSR
  *(--sp) = (uint32_t)taskHandler; // PC (start of task handler)
  *(--sp) = 0x0000000EU; // LR
  *(--sp) = 0x0000000CU; // R12
  *(--sp) = 0x00000003U; // R3
  *(--sp) = 0x00000002U; // R2
  *(--sp) = 0x00000001U; // R1
  *(--sp) = 0x00000000U; // R0
  //*(--sp) = 0xFFFFFFF9U; // PC (Special exception return instruction) see ref manual p. 20
  //*(--sp) = 0x00000000U; // R7 (0?)
  
  // Also add the extra registers to save between context switches (R4-R11)
  *(--sp) = 0x0000000BU; // R11
  *(--sp) = 0x0000000AU; // R10
  *(--sp) = 0x00000009U; // R9
  *(--sp) = 0x00000008U; // R8
  *(--sp) = 0x00000007U; // R7
  *(--sp) = 0x00000006U; // R6
  *(--sp) = 0x00000005U; // R5
  *(--sp) = 0x00000004U; // R4
  
  task->sp = sp;
  
  // round the top of stack to the next 8-byte boundary
  stack_end = (uint32_t *)(((((uintptr_t)stackLoc - 0x1U) / 8) + 0x1U) *8);
  uint32_t * testround = (uint32_t *)(((uintptr_t)stackLoc + 0x7U) & ~(0x7U));
  
  for (sp = sp - 0x1U; sp >= stack_end; sp--)
  {
    *sp = 0xDEADBEEFU;
  }
}

void OSRun(void)
{
  OSRunCallback();
  
  __disable_irq();
  OSSched();
  __enable_irq();
}

__attribute__ ((naked))
void PendSV_Handler(void)
//void OSPendSV(void)
{
  /*// dummy sp for stack loc
    void *sp;
  
  if (OSCurrent != (OSTask *)0)
  {
    //push extra registers onto the stack
    OSCurrent->sp = sp;
  }
  sp = OSNext->sp;
  
  OSCurrent = OSNext;
  
  //then, pop registers
  */
  
__asm volatile (
    //{   
//0x08000738 B081      SUB      sp,sp,#0x04
// __disable_irq();
  "CPSID         I                  \n"  
//  if (OSCurrent != (OSTask *)0) 
    //  { 
  "LDR      r1,=OSCurrent   ;@0x08000784 \n"
  "LDR      r1,[r1,#0x00] \n"
  "CMP      r1,#0x00    \n"
  "BEQ      load_next \n"
//0x08000742 E7FF      B        0x08000744

    //     push extra registers onto the stack 
  "SUB      sp,sp,#(8*4)                \n"
  "MOV      r0,sp                       \n"
  "STMIA    r0!,{r4-r7}                 \n"
  "MOV      r4,r8                       \n"
  "MOV      r5,r9                       \n"
  "MOV      r6,r10                      \n"
  "MOV      r7,r11                      \n"
  "STMIA    r0!,{r4-r7}                 \n"

    //     OSCurrent->sp = sp; 
  "MOV      r0,sp                       \n"
  "LDR      r1,=OSCurrent               \n"
  "LDR      r1,[r1,#0x00]               \n"
  "STR      r0,[r1,#0x00]               \n"
    //  } 

  "load_next:                           \n"
    //  sp = OSNext->sp;    
  "LDR      r1,=OSNext                  \n"
  "LDR      r1,[r1,#0x00]               \n"
  "LDR      r0,[r1,#0x00]               \n"
  "MOV      sp,r0                       \n"
  
    //  OSCurrent = OSNext;   
  "LDR      r1,=OSNext                  \n"
  "LDR      r1,[r1,#0x00]               \n"
  "LDR      r2,=OSCurrent               \n"
  "STR      r1,[r2,#0x00]               \n"
    //  then, pop registers
    
  "MOV      r0,sp                       \n" // r0 := top of stack
  "MOV      r2,r0                       \n"
  "ADDS     r2,r2,#(4*4)                \n" // point r2 to the 4 high registers r7-r11
  "LDMIA    r2!,{r4-r7}                 \n" // pop the 4 high registers into low registers
  "MOV      r8,r4                       \n" // move low registers into high registers
  "MOV      r9,r5                       \n"
  "MOV      r10,r6                      \n"
  "MOV      r11,r7                      \n"
  "LDMIA    r0!,{r4-r7}                 \n" // pop the low registers
  "ADD      sp,sp,#(8*4)                \n" // remove 8 registers from the stack
  
  "CPSIE    I                           \n"
  
  "BX       lr                          \n"
    //} 
//0x0800075C B001      ADD      sp,sp,#0x04
//0x0800075E 4770      BX       lr
  );
}

// actual c to asm
/**
    64: { 
    65:   // dummy sp for stack loc 
    66:     void *sp; 
    67:    
0x08000738 B081      SUB      sp,sp,#0x04
    68:   if (OSCurrent != (OSTask *)0) 
    69:   { 
    70:     //push extra registers onto the stack 
0x0800073A 4812      LDR      r0,[pc,#72]  ; @0x08000784
0x0800073C 6800      LDR      r0,[r0,#0x00]
0x0800073E 2800      CMP      r0,#0x00
0x08000740 D005      BEQ      0x0800074E
0x08000742 E7FF      B        0x08000744
    71:     OSCurrent->sp = sp; 
0x08000744 9800      LDR      r0,[sp,#0x00]
0x08000746 490F      LDR      r1,[pc,#60]  ; @0x08000784
0x08000748 6809      LDR      r1,[r1,#0x00]
0x0800074A 6008      STR      r0,[r1,#0x00]
    72:   } 
0x0800074C E7FF      B        0x0800074E
    73:   sp = OSNext->sp; 
    74:    
0x0800074E 480C      LDR      r0,[pc,#48]  ; @0x08000780
0x08000750 6801      LDR      r1,[r0,#0x00]
0x08000752 6809      LDR      r1,[r1,#0x00]
0x08000754 9100      STR      r1,[sp,#0x00]
    75:   OSCurrent = OSNext; 
    76:    
    77:   //then, pop registers 
    78:    
0x08000756 6800      LDR      r0,[r0,#0x00]
0x08000758 490A      LDR      r1,[pc,#40]  ; @0x08000784
0x0800075A 6008      STR      r0,[r1,#0x00]
    79: } 
0x0800075C B001      ADD      sp,sp,#0x04
0x0800075E 4770      BX       lr
*/
