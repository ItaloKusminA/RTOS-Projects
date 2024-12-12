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
#include "miros.h"
#include "qassert.h"
#include "stm32f1xx.h"
#include "stm32f1xx_hal.h"
Q_DEFINE_THIS_FILE

#define MAX_PERIODIC_THREADS 32
#define MAX_APERIODIC_THREADS 32
#define ROUND_UP(x) ((uint32_t)((x) + 0.999999))

OSThread * volatile OS_curr; /* pointer to the current thread */
OSThread * volatile OS_next; /* pointer to the next thread to run */
OSThread * volatile next;
OSThread * volatile lastAperiodicOccurence;


OSThread *OS_PThread[MAX_PERIODIC_THREADS + 1];/* array of threads started so far */
uint8_t OS_PreadyIndex[MAX_PERIODIC_THREADS + 1]; /* bitmask of threads that are ready to run */
uint8_t OS_Pindex = 0;
OSThread *OS_APThread[MAX_APERIODIC_THREADS];
uint8_t OS_APreadyIndex[MAX_APERIODIC_THREADS]; /* bitmask of threads that are ready to run */
uint8_t OS_APindex = 0;
bool firstAperiodicOccurence = true;
double Us; //Server utilization

OSThread idleThread;
uint32_t stack_idleThread[256];
periodicParameters ITP;
aperiodicParameters ITAP;

void main_idleThread() {
    while (1) {
    	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
        OS_onIdle();
    }
}

void OS_init() {
    /* set the PendSV interrupt priority to the lowest level 0xFF */
    *(uint32_t volatile *)0xE000ED20 |= (0xFFU << 16);
    idleThread.PP = &ITP;
    idleThread.AP = &ITAP;
    /* start idleThread thread */
    OSThread_start(&idleThread,
                   UINT32_MAX,
				   UINT32_MAX,
                   &main_idleThread,
				   stack_idleThread, (sizeof(stack_idleThread)));
}

void OS_sched(void) {
	Q_ASSERT(next != (OSThread *)0);
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

void OS_run(double Up) {
    Us = 1 - Up;
    uint8_t Pearliest_deadline_index = OS_EarliestPeriodicDeadline();
    uint8_t APearliest_deadline_index = 0;

    if (OS_AperiodicTaskAvailable()) {
        APearliest_deadline_index = OS_EarliestAperiodicDeadline();
    }

    if (OS_PThread[Pearliest_deadline_index]->PP->deadline < OS_APThread[APearliest_deadline_index]->AP->deadline) {
        next = OS_PThread[Pearliest_deadline_index];
    } else {
        next = OS_APThread[APearliest_deadline_index];
    }

    OS_onStartup();
    __disable_irq();
    OS_sched();
    __enable_irq();

    Q_ERROR();
}

bool OS_AperiodicTaskAvailable(void){
	for (int i = 0; i < OS_APindex; i++) {
		if (OS_APreadyIndex[i] == 1) return true;
	}
	return false;
}

uint32_t OS_EarliestAperiodicDeadline(void){
	uint8_t earliest_deadline_index = 0;
	for (int i = 0; i < OS_APindex; i++) {
		if (OS_APThread[i]) {
			if (OS_APreadyIndex[i] == 0){
			} else {
				if (OS_APThread[i]->AP->deadline < OS_APThread[earliest_deadline_index]->AP->deadline) {
					earliest_deadline_index = i;
				}
			}
			if(OS_APThread[i]->NPPprio == UINT32_MAX){
				return i;
			}
			if(OS_APThread[i]->AP->deadline == 0){
				OS_APreadyIndex[OS_APThread[i]->index] = 0;
			}
		}
	}
	return earliest_deadline_index;
}

uint32_t OS_EarliestPeriodicDeadline(void){
	bool idleVerification = false;
	uint8_t earliest_deadline_index = 0;
	for (int i = 1; i < OS_Pindex; i++) {
		if (OS_PThread[i]) {
			if (OS_PreadyIndex[i] == 0){
			} else {
				idleVerification = true;
				if (OS_PThread[i]->PP->deadline < OS_PThread[earliest_deadline_index]->PP->deadline) {
					earliest_deadline_index = i;
				}
			}
			if(OS_PThread[i]->NPPprio == UINT32_MAX){
				return i;
			}
		}
	}
	if (!idleVerification) return 0;
	return earliest_deadline_index;
}

void OS_tick(void) {
    uint8_t Pi = OS_EarliestPeriodicDeadline();
    if (OS_AperiodicTaskAvailable()) {
        uint8_t APi = OS_EarliestAperiodicDeadline();
        if (OS_PThread[Pi]->PP->deadline < OS_APThread[APi]->AP->deadline) {
            next = OS_PThread[Pi];
            if(OS_APThread[APi]->NPPprio == UINT32_MAX) next = OS_APThread[APi];
        } else {
            next = OS_APThread[APi];
            if(OS_PThread[Pi]->NPPprio == UINT32_MAX) next = OS_PThread[Pi];
        }
    } else {
        next = OS_PThread[Pi];
    }

    for (int i = 1; i < OS_Pindex; i++) {
        if (OS_PThread[i]) {
            if (OS_PreadyIndex[i] == 0) {
                OS_PThread[i]->timeout--;
                if (OS_PThread[i]->timeout == 0) {
                    OS_PreadyIndex[i] = 1;
                    OS_PThread[i]->PP->deadline = OS_PThread[i]->PP->RELdeadline;
                    OS_PThread[i]->PP->period = OS_PThread[i]->PP->RELperiod;
                }
            }
            else{
            	OS_PThread[i]->PP->period--;
            	OS_PThread[i]->PP->deadline--;
            }
        }
    }

    if (OS_AperiodicTaskAvailable()) {
        for (int i = 0; i < OS_APindex; i++) {
            if (OS_APThread[i]) {
                if (OS_APreadyIndex[i] == 0) {
                }
                else{
					OS_APThread[i]->AP->deadline--;
					if(OS_APThread[i]->AP->deadline == 0)OS_APreadyIndex[i] = 0;
                }
            }
         }
    }
}

void OS_error(void){
	while (1){

	}
}
void OS_delay(uint32_t ticks) {
    __asm volatile ("cpsid i");
    /* never call OS_delay from the idleThread */
    Q_REQUIRE(OS_curr != OS_PThread[0]);
    OS_curr->timeout = ticks;
    if(!OS_curr->AP)
    	OS_PreadyIndex[OS_curr->index] = 0;
    if(!OS_curr->PP)
        OS_APreadyIndex[OS_curr->index] = 0;
    __disable_irq();
    OS_sched();
    __enable_irq();
    __asm volatile ("cpsie i");
}

void OS_yield(){
	OS_delay(1);
}

void OS_TBS(OSThread *occurenceTask){
	if(OS_APreadyIndex[occurenceTask->index] == 0){
		if(firstAperiodicOccurence == true){
			occurenceTask->AP->deadline = ROUND_UP((double)occurenceTask->AP->computationTime / Us);
			firstAperiodicOccurence = false;
		}
		else{
			if(OS_AperiodicTaskAvailable()){
				occurenceTask->AP->deadline = lastAperiodicOccurence->AP->deadline + ROUND_UP((double)occurenceTask->AP->computationTime / Us);
			}
			else{
				occurenceTask->AP->deadline = ROUND_UP((double)occurenceTask->AP->computationTime / Us);
			}
		}
		lastAperiodicOccurence = occurenceTask;
		OS_APreadyIndex[occurenceTask->index] = 1;
	}
}

void OS_waitNextOccurence(void){
	__disable_irq();
	OS_APreadyIndex[OS_curr->index] = 0;
	OS_APThread[OS_curr->index]->AP->deadline = UINT32_MAX;
	if(OS_AperiodicTaskAvailable()){
		next = OS_APThread[OS_EarliestAperiodicDeadline()];
		if (OS_PThread[OS_EarliestPeriodicDeadline()]->PP->deadline < next->AP->deadline){
			next = OS_PThread[OS_EarliestPeriodicDeadline()];
			if(OS_APThread[OS_EarliestAperiodicDeadline()]->NPPprio == UINT32_MAX)next = OS_APThread[OS_EarliestAperiodicDeadline()];
		}
	}
	else{
		next = OS_PThread[OS_EarliestPeriodicDeadline()];
	}
    OS_sched();
    __enable_irq();
}

void OS_waitNextPeriod(void){
	__disable_irq();
	OS_PreadyIndex[OS_curr->index] = 0;
	OS_curr->timeout = OS_curr->PP->period;
	if(OS_AperiodicTaskAvailable()){
		next = OS_APThread[OS_EarliestAperiodicDeadline()];
		if (OS_PThread[OS_EarliestPeriodicDeadline()]->PP->deadline < next->AP->deadline){
			next = OS_PThread[OS_EarliestPeriodicDeadline()];
			if(OS_APThread[OS_EarliestAperiodicDeadline()]->NPPprio == UINT32_MAX)next = OS_APThread[OS_EarliestAperiodicDeadline()];
		}
	}
	else{
		next = OS_PThread[OS_EarliestPeriodicDeadline()];
	}
    OS_sched();
    __enable_irq();
}

void OSAperiodic_thread_start(
    OSThread *me,
	uint32_t computationTime,
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
    //me->PP = NULL;
    me->AP->computationTime = computationTime;
    me->AP->deadline = UINT32_MAX;
    me->index = OS_APindex;
    me->criticalRegionHistoric = 0;
    me->timeout = 0;
    me->NPPprio = 0;
    OS_APThread[OS_APindex] = me;
    OS_APreadyIndex[OS_APindex] = 0;
    OS_APindex++;

    /* round up the bottom of the stack to the 8-byte boundary */
    stk_limit = (uint32_t *)(((((uint32_t)stkSto - 1U) / 8) + 1U) * 8);

    /* pre-fill the unused part of the stack with 0xDEADBEEF */
    for (sp = sp - 1U; sp >= stk_limit; --sp) {
        *sp = 0xDEADBEEFU;
    }
}

void OSThread_start(
    OSThread *me,
    uint32_t relativeDeadline, /* task deadline */
	uint32_t relativePeriod,
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
    //me->AP = NULL;
    me->PP->RELdeadline = relativeDeadline;
    me->PP->deadline = relativeDeadline;
    me->PP->RELperiod = relativePeriod;
    me->PP->period = relativePeriod;
    me->index = OS_Pindex;
    me->criticalRegionHistoric = 0;
    me->NPPprio = 0;
    OS_PThread[OS_Pindex] = me;
    OS_PreadyIndex[OS_Pindex] = 1;
    OS_Pindex++;

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
	OS_curr->NPPprio = UINT32_MAX;
	OS_curr->criticalRegionHistoric++;
	semaphore->current_value--;
	__enable_irq();
}

void semaphore_post(semaphore_t* semaphore){
	Q_ASSERT(semaphore);
	__disable_irq();
	OS_curr->criticalRegionHistoric--;
	if(OS_curr->criticalRegionHistoric == 0){
		OS_curr->NPPprio = 0;
	}
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
