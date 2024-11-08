#include <stdint.h>
#include <stdlib.h>
#include <time.h>
#include "miros.h"
#include "stm32f1xx_hal.h"

#define SENS_PERIOD 2
#define SENS_DEADLINE 2

#define CTRL_PERIOD 5
#define CTRL_DEADLINE 5

#define DIAG_PERIOD 500
#define DIAG_DEADLINE 500

typedef struct {
    OSThread taskT;
    uint32_t stackT[40];
} tasks_t;

tasks_t taskSensoring;
tasks_t taskControl;
tasks_t taskDiagnostics;

volatile int crankshaftPosition;
volatile int massAirFlow;
volatile int engineTemperature;
volatile int oxygenSensor;
volatile int iteration = 0;
float executionTimeAverage;

int readCrankshaftPosition() {
    return rand() % 360;
}

int readMassAirFlow() {
    return rand() % 100;
}

int readEngineTemperature() {
    return rand() % 120;
}

int readOxygenSensor() {
    return rand() % 100;
}

int calculateInjectionTime(int crankshaftPosition, int massAirFlow) {
    // Illustrative function
    return crankshaftPosition * massAirFlow;
}

int calculateIgnitionAdvance(int crankshaftPosition, int engineTemperature) {
    // Illustrative function
    return crankshaftPosition * engineTemperature;
}

void controlInjectors(int injectionTime) {
    // Illustrative function
}

void controlIgnitionSystem(int ignitionAdvance) {
    // Illustrative function
}

void monitorSensors() {
    // Illustrative function
}

void monitorActuators() {
    // Illustrative function
}

void detectFaults() {
    // Illustrative function
}

void TaskSensoring() {
    while (1) {
        //int start_tick = HAL_GetTick();
        crankshaftPosition = readCrankshaftPosition();
        massAirFlow = readMassAirFlow();
        engineTemperature = readEngineTemperature();
        oxygenSensor = readOxygenSensor();
        //int ending_tick = HAL_GetTick();
        //int execution_time = ending_tick - start_tick;
        //iteration++;
        //executionTimeAverage = ((executionTimeAverage * (iteration - 1)) + execution_time) / iteration;
        OS_waitNextPeriod();
    }
}

void TaskControl() {
    while (1) {
        //int start_tick = HAL_GetTick();
        int injectionTime = calculateInjectionTime(crankshaftPosition, massAirFlow);
        int ignitionAdvance = calculateIgnitionAdvance(crankshaftPosition, engineTemperature);
        controlInjectors(injectionTime);
        controlIgnitionSystem(ignitionAdvance);
        //int ending_tick = HAL_GetTick();
        //int execution_time = ending_tick - start_tick;
        //iteration++;
        //executionTimeAverage = ((executionTimeAverage * (iteration - 1)) + execution_time) / iteration;
        OS_waitNextPeriod();
    }
}

void TaskDiagnostics() {
    while (1) {
        //int start_tick = HAL_GetTick();
        monitorSensors();
        monitorActuators();
        detectFaults();
        //int ending_tick = HAL_GetTick();
        //int execution_time = ending_tick - start_tick;
        //iteration++;
        //executionTimeAverage = ((executionTimeAverage * (iteration - 1)) + execution_time) / iteration;
        OS_waitNextPeriod();
    }
}

int main() {
    srand(time(NULL));

    OS_init();

    OSThread_start(
        &taskSensoring.taskT,
        SENS_DEADLINE,
        SENS_PERIOD,
        &TaskSensoring,
        &taskSensoring.stackT, sizeof(taskSensoring.stackT));

    OSThread_start(
        &taskControl.taskT,
        CTRL_DEADLINE,
        CTRL_PERIOD,
        &TaskControl,
        &taskControl.stackT, sizeof(taskControl.stackT));

    OSThread_start(
        &taskDiagnostics.taskT,
        DIAG_DEADLINE,
        DIAG_PERIOD,
        &TaskDiagnostics,
        &taskDiagnostics.stackT, sizeof(taskDiagnostics.stackT));

    OS_run();

    return 0;
}
