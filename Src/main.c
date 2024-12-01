#include <stdint.h>
#include <stdlib.h>
#include <time.h>
#include "miros.h"
#include "stm32f1xx_hal.h"


//Em millis
#define SENS_PERIOD 100
#define SENS_DEADLINE 100
#define SENS_WCCT 50

#define CTRL_PERIOD 500
#define CTRL_DEADLINE 500
#define CTRL_WCCT 100

#define DIAG_WCCT 30


typedef struct{
    OSThread taskT;
    uint32_t stackT[40];
} tasks_t;

periodicParameters TSPP;
aperiodicParameters TSAP;
periodicParameters TCPP;
aperiodicParameters TCAP;
aperiodicParameters TDAP;
periodicParameters TDPP;

tasks_t taskSensoring;
tasks_t taskControl;
tasks_t taskDiagnostics;

semaphore_t criticalRegionA;
semaphore_t criticalRegionB;
semaphore_t criticalRegionC;


volatile int crankshaftPosition;
volatile int massAirFlow;
volatile int engineTemperature;
volatile int oxygenSensor;
volatile int iteration = 0;
volatile int a = 0;
volatile int b = 0;
volatile int c = 0;

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
    	semaphore_wait(&criticalRegionA);
        //int start_tick = HAL_GetTick();
        crankshaftPosition = readCrankshaftPosition();
        massAirFlow = readMassAirFlow();
        engineTemperature = readEngineTemperature();
        oxygenSensor = readOxygenSensor();
        a++;
        //int ending_tick = HAL_GetTick();
        //int execution_time = ending_tick - start_tick;
        //iteration++;
        //executionTimeAverage = ((executionTimeAverage * (iteration - 1)) + execution_time) / iteration;
        semaphore_post(&criticalRegionA);
        OS_waitNextPeriod();
    }
}

void TaskControl() {
    while (1) {
    	semaphore_wait(&criticalRegionB);
        //int start_tick = HAL_GetTick();
        int injectionTime = calculateInjectionTime(crankshaftPosition, massAirFlow);
        int ignitionAdvance = calculateIgnitionAdvance(crankshaftPosition, engineTemperature);
        controlInjectors(injectionTime);
        controlIgnitionSystem(ignitionAdvance);
        b++;
        //int ending_tick = HAL_GetTick();
        //int execution_time = ending_tick - start_tick;
        //iteration++;
        //executionTimeAverage = ((executionTimeAverage * (iteration - 1)) + execution_time) / iteration;
        semaphore_post(&criticalRegionB);
        OS_waitNextPeriod();
    }
}

void TaskDiagnostics() {
    while (1) {
        //int start_tick = HAL_GetTick();
    	semaphore_wait(&criticalRegionC);
        monitorSensors();
        monitorActuators();
        detectFaults();
        c++;
        //int ending_tick = HAL_GetTick();
        //int execution_time = ending_tick - start_tick;
        //iteration++;
        //executionTimeAverage = ((executionTimeAverage * (iteration - 1)) + execution_time) / iteration;
        semaphore_post(&criticalRegionC);
        OS_waitNextOccurence();
    }
}
uint32_t previousTick = 0;

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	uint32_t currentTick = HAL_GetTick();
	if (GPIO_Pin == GPIO_PIN_0 && (currentTick - previousTick) > 20){
		previousTick = currentTick;
		OS_TBS(&taskDiagnostics.taskT);
		iteration++;
	}
}

static void MX_GPIO_Init(void);

int main() {
	HAL_Init();
	MX_GPIO_Init();

	taskSensoring.taskT.PP = &TSPP;
	taskSensoring.taskT.AP = &TSAP;
	taskControl.taskT.PP = &TCPP;
	taskControl.taskT.AP = &TCAP;
	taskDiagnostics.taskT.PP = &TDPP;
	taskDiagnostics.taskT.AP = &TDAP;
	//aperiodicHandler.taskT.PP = &AHPP;
	semaphore_init(&criticalRegionA, 1);
	semaphore_init(&criticalRegionB, 1);
	semaphore_init(&criticalRegionC, 1);

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

    OSAperiodic_thread_start(
        &taskDiagnostics.taskT,
        DIAG_WCCT,
        &TaskDiagnostics,
        &taskDiagnostics.stackT, sizeof(taskDiagnostics.stackT));

    double Up = ((double)SENS_WCCT / (double)SENS_PERIOD) +
    			//((double)APH_WCCT / (double)APH_PERIOD) +
                ((double)CTRL_WCCT / (double)CTRL_PERIOD);

    OS_run(Up);

    return 0;
}

static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOB_CLK_ENABLE();

  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0U, 0U);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);
}
