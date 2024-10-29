/****************************************************************************
* MInimal Real-time Operating System (MiROS), GNU-ARM port.
* version 1.26 (matching lesson 26, see https://youtu.be/kLxxXNCrY60)
*
* This software is a teaching aid to illustrate the concepts underlying
* a Real-Time Operating System (RTOS). The main goal of the software is
* simplicity and clear presentation of the concepts, but without dealing
* with various corner cases, portability, or error handling. For these
* reasons, the software is generally NOT intended or recommended for use
* in commercial applications.
*
* Copyright (C) 2018 Miro Samek. All Rights Reserved.
*
* SPDX-License-Identifier: GPL-3.0-or-later
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program. If not, see <https://www.gnu.org/licenses/>.
*
* Git repo:
* https://github.com/QuantumLeaps/MiROS
****************************************************************************/
#include <stdint.h>
#include <stdbool.h>
#include "miros.h"
#include "qassert.h"
#include "stm32f1xx.h"
#include "stm32f1xx_hal.h"
Q_DEFINE_THIS_FILE

#define MAX_THREADS 32

OSThread * volatile OS_curr; /* pointer to the current thread */
OSThread * volatile OS_next; /* pointer to the next thread to run */

OSThread *OS_thread[MAX_THREADS + 1]; /* array of threads started so far */
uint8_t OS_readyIndex[MAX_THREADS + 1]; /* bitmask of threads that are ready to run */
uint8_t OS_threadIndex = 0;
uint8_t sched_index;

OSThread idleThread;
uint32_t stack_idleThread[40];

void main_idleThread() {
    while (1) {
        OS_onIdle();
    }
}

void OS_init() {
    /* set the PendSV interrupt priority to the lowest level 0xFF */
    *(uint32_t volatile *)0xE000ED20 |= (0xFFU << 16);

    /* start idleThread thread */
    OSThread_start(&idleThread,
                   UINT32_MAX,
				   UINT32_MAX,
				   0U,
                   &main_idleThread,
				   stack_idleThread, (sizeof(stack_idleThread)));
}

void OS_sched(void) {
	OSThread *next;

	    if (sched_index == 0U) { /* idle condition? */
	        next = OS_thread[0]; /* the idle thread */
	    }
	    else {
	        next = OS_thread[sched_index];
	        Q_ASSERT(next != (OSThread *)0);
	    }
    /* trigger PendSV, if needed */
    if (next != OS_curr) {
        OS_next = next;
        //*(uint32_t volatile *)0xE000ED04 = (1U << 28);
        SCB->ICSR |= SCB_ICSR_PENDSVSET_Msk;
        __asm volatile("dsb");
//        __asm volatile("isb");
    }
    /*
     * DSB - whenever a memory access needs to have completed before program execution progresses.
     * ISB - whenever instruction fetches need to explicitly take place after a certain point in the program,
     * for example after memory map updates or after writing code to be executed.
     * (In practice, this means "throw away any prefetched instructions at this point".)
     * */
}

void OS_run(void) {
    /* callback to configure and start interrupts */
	bool idleVerification = false;
	int earliest_deadline_index = 0;
	for(int i = 1; i < OS_threadIndex+1; i++){
		if(OS_thread[i]){
			if(OS_readyIndex[i] == 0){
			}
			else{
				if (OS_thread[i]->deadline < OS_thread[earliest_deadline_index]->deadline) {
					earliest_deadline_index = i;
				}
				idleVerification = true;
			}
		}
	}
	if(!idleVerification) sched_index = 0;
    OS_onStartup();
    __disable_irq();
    OS_sched();
    __enable_irq();

    /* the following code should never execute */
    Q_ERROR();
}

void OS_tick(void) {
    bool idleVerification = false;
    uint8_t earliest_deadline_index = 0;
    for (int i = 1; i < OS_threadIndex+1; i++) {
        if (OS_thread[i]) {
        	if (OS_readyIndex[i] == 0){
				if (OS_thread[i]->timeout == 0) {
					OS_readyIndex[i] = 1;
					OS_thread[i]->deadline = OS_thread[i]->ABSdeadline;
					OS_thread[i]->computation_time = OS_thread[i]->ABScomputation_time;
					OS_thread[i]->period = OS_thread[i]->ABSperiod;
				}
				OS_thread[i]->timeout--;
        	} else {
        		idleVerification = true;
        		if (OS_thread[i]->deadline < OS_thread[earliest_deadline_index]->deadline) {
        			earliest_deadline_index = i;
        		}
        		if(OS_thread[i]->computation_time == 0){
        			OS_readyIndex[i] = 0;
        			OS_thread[i]->timeout = OS_thread[i]->period;
        		}
        		OS_thread[i]->period--;
        	}
        }
        OS_thread[i]->deadline--;
    }
    OS_thread[earliest_deadline_index]->computation_time--;
    sched_index = earliest_deadline_index;
    if (!idleVerification) sched_index = 0;
}

void OS_delay(uint32_t ticks) {
    __asm volatile ("cpsid i");

    /* never call OS_delay from the idleThread */
    Q_REQUIRE(OS_curr != OS_thread[0]);
    OS_curr->timeout = ticks;
    OS_readyIndex[OS_curr->index] = 0;
    OS_sched();
    __asm volatile ("cpsie i");
}

void OS_yield(){
	OS_delay(1);
}

void OSThread_start(
    OSThread *me,
    uint32_t absolute_deadline, /* task deadline */
	uint32_t absolute_period,
	uint32_t absolute_computation_time,
    OSThreadHandler threadHandler,
    void *stkSto, uint32_t stkSize)
{
    /* round down the stack top to the 8-byte boundary
    * NOTE: ARM Cortex-M stack grows down from hi -> low memory
    */
    uint32_t *sp = (uint32_t *)((((uint32_t)stkSto + stkSize) / 8) * 8);
    uint32_t *stk_limit;


    *(--sp) = (1U << 24);  /* xPSR */
    *(--sp) = (uint32_t)threadHandler; /* PC */
    *(--sp) = 0x0000000EU; /* LR  */
    *(--sp) = 0x0000000CU; /* R12 */
    *(--sp) = 0x00000003U; /* R3  */
    *(--sp) = 0x00000002U; /* R2  */
    *(--sp) = 0x00000001U; /* R1  */
    *(--sp) = 0x00000000U; /* R0  */
    /* additionally, fake registers R4-R11 */
    *(--sp) = 0x0000000BU; /* R11 */
    *(--sp) = 0x0000000AU; /* R10 */
    *(--sp) = 0x00000009U; /* R9 */
    *(--sp) = 0x00000008U; /* R8 */
    *(--sp) = 0x00000007U; /* R7 */
    *(--sp) = 0x00000006U; /* R6 */
    *(--sp) = 0x00000005U; /* R5 */
    *(--sp) = 0x00000004U; /* R4 */

    /* save the top of the stack in the thread's attibute */
    me->sp = sp;
    me->ABScomputation_time = absolute_computation_time;
    me->computation_time = absolute_computation_time;
    me->ABSdeadline = absolute_deadline;
    me->deadline = absolute_deadline;
    me->ABSperiod = absolute_period;
    me->period = absolute_period;
    me->index = OS_threadIndex;
    OS_thread[OS_threadIndex] = me;
    OS_readyIndex[OS_threadIndex] = 1;
    OS_threadIndex++;

    /* round up the bottom of the stack to the 8-byte boundary */
    stk_limit = (uint32_t *)(((((uint32_t)stkSto - 1U) / 8) + 1U) * 8);

    /* pre-fill the unused part of the stack with 0xDEADBEEF */
    for (sp = sp - 1U; sp >= stk_limit; --sp) {
        *sp = 0xDEADBEEFU;
    }
}

void semaphore_init(semaphore_t* semaphore, uint8_t starting_value){
	Q_ASSERT(semaphore);
	semaphore->current_value = starting_value;
}

void semaphore_wait(semaphore_t* semaphore){
	Q_ASSERT(semaphore);
	__disable_irq();
	while (semaphore->current_value == 0){
		OS_yield(1);
		__disable_irq();
	}
	semaphore->current_value--;
	__enable_irq();
}

void semaphore_post(semaphore_t* semaphore){
	Q_ASSERT(semaphore);
	__disable_irq();
	semaphore->current_value++;
	__enable_irq();

}

__attribute__ ((naked, optimize("-fno-stack-protector")))
void PendSV_Handler(void) {
__asm volatile (

    /* __disable_irq(); */
    "  CPSID         I                 \n"

    /* if (OS_curr != (OSThread *)0) { */
    "  LDR           r1,=OS_curr       \n"
    "  LDR           r1,[r1,#0x00]     \n"
    "  CBZ           r1,PendSV_restore \n"

    /*     push registers r4-r11 on the stack */
    "  PUSH          {r4-r11}          \n"

    /*     OS_curr->sp = sp; */
    "  LDR           r1,=OS_curr       \n"
    "  LDR           r1,[r1,#0x00]     \n"
    "  STR           sp,[r1,#0x00]     \n"
    /* } */

    "PendSV_restore:                   \n"
    /* sp = OS_next->sp; */
    "  LDR           r1,=OS_next       \n"
    "  LDR           r1,[r1,#0x00]     \n"
    "  LDR           sp,[r1,#0x00]     \n"

    /* OS_curr = OS_next; */
    "  LDR           r1,=OS_next       \n"
    "  LDR           r1,[r1,#0x00]     \n"
    "  LDR           r2,=OS_curr       \n"
    "  STR           r1,[r2,#0x00]     \n"

    /* pop registers r4-r11 */
    "  POP           {r4-r11}          \n"

    /* __enable_irq(); */
    "  CPSIE         I                 \n"

    /* return to the next thread */
    "  BX            lr                \n"
    );
}
