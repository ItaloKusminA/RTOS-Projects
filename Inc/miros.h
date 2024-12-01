/****************************************************************************
* MInimal Real-time Operating System (MiROS)
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
#ifndef MIROS_H
#define MIROS_H

#include <stdbool.h>

typedef struct aperiodicParameters {
	uint32_t computationTime;
	uint32_t deadline; /* thread priority */
}aperiodicParameters;

typedef struct periodicParameters {
    uint32_t RELdeadline; /* thread priority */
    uint32_t RELperiod;
    uint32_t deadline; /* thread priority */
    uint32_t period;
}periodicParameters;

/* Thread Control Block (TCB) */
typedef struct OSThread {
    void *sp; /* stack pointer */
    uint32_t timeout;
    uint32_t index;
    aperiodicParameters *AP;
	periodicParameters *PP;
	uint32_t NPPprio;
	uint32_t criticalRegionHistoric;
    /* ... other attributes associated with a thread */
} OSThread;

#define TICKS_PER_SEC 1000U

typedef void (*OSThreadHandler)();

void OS_init(void);

/* callback to handle the idle condition */
void OS_onIdle(void);

/* this function must be called with interrupts DISABLED */
void OS_sched(void);

/* transfer control to the RTOS to run the threads */
void OS_run(double Up);

bool OS_AperiodicTaskAvailable(void);

uint32_t OS_EarliestAperiodicDeadline(void);

uint32_t OS_EarliestPeriodicDeadline(void);

/* blocking delay */
void OS_delay(uint32_t ticks);

/* process all timeouts */
void OS_tick(void);

/* callback to configure and start interrupts */
void OS_onStartup(void);

void OS_TBS(OSThread *occurenceTask);

void OS_waitNextOccurence(void);

void OS_waitNextPeriod(void);

void OSAperiodic_thread_start(
    OSThread *me,
	uint32_t computationTime,
    OSThreadHandler threadHandler,
    void *stkSto, uint32_t stkSize);

void OSThread_start(
    OSThread *me,
    uint32_t relativeDeadline, /* task deadline */
	uint32_t relativePeriod,
    OSThreadHandler threadHandler,
    void *stkSto, uint32_t stkSize);

typedef struct {
	uint8_t current_value;
} semaphore_t;

void semaphore_init(semaphore_t* semaphore, uint8_t starting_value);

void semaphore_wait(semaphore_t* semaphore);

void semaphore_post(semaphore_t* semaphore);


#endif /* MIROS_H */
