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

tasks_t taskSensoriamento;
tasks_t taskControle;
tasks_t taskDiagnostico;

volatile int crankshaftPosition;
volatile int massAirFlow;
volatile int engineTemperature;
volatile int oxygenSensor;


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
	//Ilustrative function
	return crankshaftPosition*massAirFlow;
}

int calculateIgnitionAdvance(int crankshaftPosition, int engineTemperature) {
	//Ilustrative function
	return crankshaftPosition*engineTemperature;
}

void controlInjectors(int injectionTime) {
	//Ilustrative function
}

void controlIgnitionSystem(int ignitionAdvance) {
	//Ilustrative function
}


void monitorSensors() {
	//Ilustrative function
}

void monitorActuators() {
	//Ilustrative function
}

void detectFaults() {
	//Ilustrative function
}

void TaskSensoriamento() {
	while(1){
    	crankshaftPosition = readCrankshaftPosition();
        massAirFlow = readMassAirFlow();
        engineTemperature = readEngineTemperature();
        oxygenSensor = readOxygenSensor();
        OS_waitNextPeriod();
	}
}

void TaskControle() {
	while(1){
		int injectionTime = calculateInjectionTime(crankshaftPosition, massAirFlow);
		int ignitionAdvance = calculateIgnitionAdvance(crankshaftPosition, engineTemperature);
		controlInjectors(injectionTime);
		controlIgnitionSystem(ignitionAdvance);
		OS_waitNextPeriod();
	}
}

void TaskDiagnostico() {
	while(1){
		monitorSensors();
		monitorActuators();
		detectFaults();
		OS_waitNextPeriod();
	}

}

int main() {
    srand(time(NULL));

    OS_init();

    OSThread_start(
        &taskSensoriamento.taskT,
        SENS_DEADLINE,
        SENS_PERIOD,
        &TaskSensoriamento,
        &taskSensoriamento.stackT, sizeof(taskSensoriamento.stackT));

    OSThread_start(
        &taskControle.taskT,
        CTRL_DEADLINE,
        CTRL_PERIOD,
        &TaskControle,
        &taskControle.stackT, sizeof(taskControle.stackT));

    OSThread_start(
        &taskDiagnostico.taskT,
        DIAG_DEADLINE,
        DIAG_PERIOD,
        &TaskDiagnostico,
        &taskDiagnostico.stackT, sizeof(taskDiagnostico.stackT));

    OS_run();

    return 0;
}
