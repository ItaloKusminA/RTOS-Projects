#include <stdint.h>
#include <stdlib.h>
#include <time.h>
#include "VL53L0X.h"
#include "miros.h"
#include "stm32f1xx_hal.h"

/*
Conections:

button: B0

PWM : A0

Sensor de distância:
    SCL: B6
    SDA: B7

*/

//em milis
#define CTRL_PERIOD 50
#define CTRL_DEADLINE 50
#define CTRL_WCCT 5

#define PID_PERIOD 50
#define PID_DEADLINE 50
#define PID_WCCT 25

#define READ_PERIOD 50
#define READ_DEADLINE 50
#define READ_WCCT 10

#define CH_WCCT 10

//static struct VL53L0X myTOFsensor = {.io_2v8 = false, .address = 0x29, .io_timeout = 500, .did_timeout = false};
TIM_HandleTypeDef htim2;


typedef struct{
    OSThread taskT;
    uint32_t stackT[256];
} tasks_t;

typedef struct{
	double pidValue;
	double Kp;
	double Ki;
	double Kd;
	double engineeringAdjust; // % de dutyCycle
	int error;
	double errorIntegral;
	double errorDerivative;
	int previousError;
	int setPoint;
	int measuredValue;
} PID_t;

// Initialization of periodic and aperiodic structs of each task
periodicParameters CTRLPP;
aperiodicParameters CTRLAP;
periodicParameters PIDPP;
aperiodicParameters PIDAP;
periodicParameters READPP;
aperiodicParameters READAP;
periodicParameters CHPP;
aperiodicParameters CHAP;

// Tasks intialization
tasks_t taskPID;
tasks_t taskCTRL;
tasks_t taskCH;
tasks_t taskREAD;

// Critical regions semaphores initialization
semaphore_t measuredValueSemaphore;
semaphore_t setPointSemaphore;
semaphore_t errorIntegralSemaphore;
semaphore_t pidValueSemaphore;

// PID initialization
PID_t PID;

const uint16_t period = 1000;
uint32_t currentTick = 0;

void TaskREAD() {
    while (1) {
    	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
    	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
    	//Ler o sensor com I2C
    	semaphore_wait(&measuredValueSemaphore);
    	 //PID.measuredValue = (int)VL53L0X_readRangeContinuousMillimeters(&myTOFsensor);
    	semaphore_post(&measuredValueSemaphore);
    	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
    	OS_waitNextPeriod();
    }
}

void TaskPID(){
	while(1){
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET);
		float dt = (float)((HAL_GetTick() - currentTick)/1000);
		semaphore_wait(&measuredValueSemaphore);
		semaphore_wait(&setPointSemaphore);
		PID.error = PID.setPoint - PID.measuredValue;
		semaphore_post(&setPointSemaphore);
		semaphore_post(&measuredValueSemaphore);
		PID.errorIntegral += (float)(PID.error*dt);
		PID.errorDerivative = (float)((PID.error - PID.previousError)/dt);
		PID.previousError = PID.error;
		currentTick = HAL_GetTick();
		//Calcula PID
		semaphore_wait(&pidValueSemaphore);
		PID.pidValue = (float)(PID.Kp*PID.error + PID.Ki*PID.errorIntegral + PID.Kd*PID.errorDerivative);
		if (PID.pidValue >= 0.3) PID.pidValue = 0.3;
		if (PID.pidValue <= -0.3) PID.pidValue = -0.3;
		semaphore_post(&pidValueSemaphore);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);
		OS_waitNextPeriod();
	}
}
void setPWM()
{
	TIM2->CCR1 = (int)((PID.pidValue + 0.61)*TIM2->ARR);
}

void TaskCTRL() {
    while (1) {
    	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
    	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET);
    	semaphore_wait(&pidValueSemaphore);
    	setPWM();
    	semaphore_post(&pidValueSemaphore);
    	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);
        OS_waitNextPeriod();
    }
}

void TaskCH() {
    while (1) {
    	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
    	//chamada pelo interrupt, altera o setpoint
    	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
    	semaphore_wait(&setPointSemaphore);
    	if(PID.setPoint == 400) PID.setPoint = 200;
    	else PID.setPoint = 400;
		semaphore_post(&setPointSemaphore);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
        OS_waitNextOccurence();
    }
}

volatile uint32_t previousTick = 0;

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	uint32_t currentTick = HAL_GetTick();
	if (GPIO_Pin == GPIO_PIN_0 && (currentTick - previousTick) > 20){
		previousTick = currentTick;
		OS_TBS(&taskCH.taskT);
	}
}

static void MX_GPIO_Init(void);
void MX_TIM2_Init(void);
//void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

int main() {
	HAL_Init();
	MX_TIM2_Init();
	MX_GPIO_Init();
	semaphore_init(&measuredValueSemaphore, 1);
	semaphore_init(&setPointSemaphore, 1);
	semaphore_init(&errorIntegralSemaphore, 1);
	semaphore_init(&pidValueSemaphore, 1);

	// Configure VL53L0X
	// usart_init(USART1, rcc_get_clock() / 115200);
	/*while(!VL53L0X_init(&myTOFsensor));
	VL53L0X_setMeasurementTimingBudget(&myTOFsensor, 20e3); // 20 ms
	VL53L0X_startContinuous(&myTOFsensor, 0);*/

    srand(time(NULL));

    OS_init();

	PID.pidValue = 0.0;
	PID.Kp = -0.0001;
	PID.Ki = -0.00001;
	PID.Kd = -0.00001;
	PID.engineeringAdjust = 0.61;
	PID.error = 0;
	PID.errorIntegral = 0.0;
	PID.errorDerivative = 0.0;
	PID.previousError = 0.0;
	PID.setPoint = 400;
	PID.measuredValue = 0;

    taskPID.taskT.PP = &PIDPP;
    taskPID.taskT.AP = &PIDAP;
    OSThread_start(
        &taskPID.taskT,
        PID_DEADLINE,
        PID_PERIOD,
        &TaskPID,
        &taskPID.stackT, sizeof(taskPID.stackT));

    taskREAD.taskT.PP = &READPP;
	taskREAD.taskT.AP = &READAP;
	OSThread_start(
		&taskREAD.taskT,
		READ_DEADLINE,
		READ_PERIOD,
		&TaskREAD,
		&taskREAD.stackT, sizeof(taskREAD.stackT));

	taskCTRL.taskT.PP = &CTRLPP;
	taskCTRL.taskT.AP = &CTRLAP;
    OSThread_start(
        &taskCTRL.taskT,
        CTRL_DEADLINE,
        CTRL_PERIOD,
        &TaskCTRL,
        &taskCTRL.stackT, sizeof(taskCTRL.stackT));

    taskCH.taskT.PP = &CHPP;
    taskCH.taskT.AP = &CHAP;
    OSAperiodic_thread_start(
        &taskCH.taskT,
        CH_WCCT,
        &TaskCH,
        &taskCH.stackT, sizeof(taskCH.stackT));

    double Up = ((double)PID_WCCT / (double)PID_PERIOD) +
    			((double)READ_WCCT / (double)READ_PERIOD) +
                ((double)CTRL_WCCT / (double)CTRL_PERIOD);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
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
  GPIO_InitStruct.Pull = GPIO_NOPULL;
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

/* TIM1 init function */
/* TIM1 init function */

void MX_TIM2_Init(void){

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 8-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = period;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
	  OS_error();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
	  OS_error();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
	  OS_error();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
	  OS_error();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0.5*period;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
	  OS_error();
  }
  //HAL_TIM_MspPostInit(&htim2);
}


