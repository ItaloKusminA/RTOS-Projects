#include <stdint.h>
#include <stdlib.h>
#include <time.h>
#include "VL53L0X.h"
#include "miros.h"
#include "stm32f1xx_hal.h"

//millis
#define A_PERIOD 50
#define A_DEADLINE 50
#define A_WCCT 5

#define B_PERIOD 50
#define B_DEADLINE 50
#define B_WCCT 25

#define C_PERIOD 50
#define C_DEADLINE 50
#define C_WCCT 10

#define D_WCCT 5


typedef struct{
    OSThread taskT;
    uint32_t stackT[256];
} tasks_t;

semaphore_t A;
semaphore_t B;
semaphore_t C;
semaphore_t D;

// Initialization of periodic and aperiodic structs of each task
periodicParameters APP;
aperiodicParameters AAP;
periodicParameters BPP;
aperiodicParameters BAP;
periodicParameters CPP;
aperiodicParameters CAP;
periodicParameters DPP;
aperiodicParameters DAP;

// Tasks intialization
tasks_t taskA;
tasks_t taskB;
tasks_t taskC;
tasks_t taskD;


uint32_t currentTick = 0;

void TaskA() {
    while (1) {
    	semaphore_wait(&A);
    	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
    	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
    	currentTick = HAL_GetTick();
    	while(HAL_GetTick() - currentTick <= A_WCCT){

    	}
    	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
    	semaphore_post(&A);
    	OS_waitNextPeriod();
    }
}

void TaskB(){
	while(1){
		semaphore_wait(&B);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET);
		currentTick = HAL_GetTick();
		while(HAL_GetTick() - currentTick <= B_WCCT){

		}
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);
		semaphore_post(&B);
		OS_waitNextPeriod();
	}
}

void TaskC() {
    while (1) {
    	semaphore_wait(&C);
    	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
    	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET);
    	currentTick = HAL_GetTick();
		while(HAL_GetTick() - currentTick <= C_WCCT){

		}
    	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);
    	semaphore_post(&C);
        OS_waitNextPeriod();
    }
}
int cont = 0;
void TaskD() {
    while (1) {
    	semaphore_wait(&D);
    	cont = 0;
    	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
    	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
    	currentTick = HAL_GetTick();
		while(HAL_GetTick() - currentTick <= D_WCCT){

		}
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
		cont = 1;
		semaphore_post(&D);
        OS_waitNextOccurence();
    }
}

volatile uint32_t cTick = 0;
volatile uint32_t pTick = 0;

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	uint32_t cTick = HAL_GetTick();
	if (GPIO_Pin == GPIO_PIN_0 && (cTick - pTick) > 20){
		pTick = cTick;
		OS_TBS(&taskD.taskT);
	}
}

static void MX_GPIO_Init(void);

int main() {
	HAL_Init();
	MX_GPIO_Init();
	semaphore_init(&A, 1);
	semaphore_init(&B, 1);
	semaphore_init(&C, 1);
	semaphore_init(&D, 1);

    srand(time(NULL));

    OS_init();


    taskA.taskT.PP = &APP;
    taskA.taskT.AP = &AAP;
    OSThread_start(
        &taskA.taskT,
        A_DEADLINE,
        A_PERIOD,
        &TaskA,
        &taskA.stackT, sizeof(taskA.stackT));

    taskB.taskT.PP = &BPP;
	taskB.taskT.AP = &BAP;
	OSThread_start(
		&taskB.taskT,
		B_DEADLINE,
		B_PERIOD,
		&TaskB,
		&taskB.stackT, sizeof(taskB.stackT));

	taskC.taskT.PP = &CPP;
	taskC.taskT.AP = &CAP;
    OSThread_start(
        &taskC.taskT,
        C_DEADLINE,
        C_PERIOD,
        &TaskC,
        &taskC.stackT, sizeof(taskC.stackT));

    taskD.taskT.PP = &DPP;
    taskD.taskT.AP = &DAP;
    OSAperiodic_thread_start(
        &taskD.taskT,
        D_WCCT,
        &TaskD,
        &taskD.stackT, sizeof(taskD.stackT));

    double Up = ((double)A_WCCT / (double)A_PERIOD) +
    			((double)B_WCCT / (double)A_PERIOD) +
                ((double)C_WCCT / (double)C_PERIOD);
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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA1 PA2 PA3 PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /**TIM2 GPIO Configuration
  PA0-WKUP     ------> TIM2_CH1
  */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0U, 0U);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);
}

