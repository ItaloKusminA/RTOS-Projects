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
#define CTRL_PERIOD 300
#define CTRL_DEADLINE 300
#define CTRL_WCCT 10

#define PID_PERIOD 200
#define PID_DEADLINE 200
#define PID_WCCT 10

#define READ_PERIOD 50
#define READ_DEADLINE 50
#define READ_WCCT 10

#define CH_WCCT 10

static struct VL53L0X myTOFsensor = {.io_2v8 = false, .address = 0x29, .io_timeout = 500, .did_timeout = false};
int measuredValue = 0;
TIM_HandleTypeDef htim2;


typedef struct{
    OSThread taskT;
    uint32_t stackT[256];
} tasks_t;

periodicParameters CTRLPP;
aperiodicParameters CTRLAP;
periodicParameters PIDPP;
aperiodicParameters PIDAP;
periodicParameters READPP;
aperiodicParameters READAP;
periodicParameters CHPP;
aperiodicParameters CHAP;

tasks_t taskPID;
tasks_t taskCTRL;
tasks_t taskCH;
tasks_t taskREAD;

semaphore_t A;
semaphore_t B;
semaphore_t C;
semaphore_t D;

int setPoint = 400; //mm
int error = 0;
double errorIntegral = 0.0;
double errorDerivative = 0.0;
int previousError = 0;
uint32_t currentTick = 0;
const double Kp = -0.0001;
const double Ki = -0.00001;
const double Kd = -0.00001;
const double engineeringAdjust = 0.61; // % de dutyCycle
const uint16_t period = 1000;
double pid = 0.0;
double dutyCycle = 0.0;

int cont = 0;
void TaskREAD() {
    while (1) {
    	//Ler o sensor com I2C
    	semaphore_wait(&A);
    	cont++;
    	measuredValue = (int)VL53L0X_readRangeContinuousMillimeters(&myTOFsensor);
    	semaphore_post(&A);
    	//Calcula o erro
    	OS_waitNextPeriod();
    }
}

void TaskPID(){
	while(1){
		semaphore_wait(&B);
		float dt = (float)((HAL_GetTick() - currentTick)/1000);
		error = setPoint - measuredValue;
		errorIntegral += (float)(error*dt);
		errorDerivative = (float)((error - previousError)/dt);
		previousError = error;
		currentTick = HAL_GetTick();
		//Calcula PID
		pid = (float)(Kp*error + Ki*errorIntegral + Kd*errorDerivative);
		if (pid >= 0.3) pid = 0.3;
		if (pid <= -0.3) pid = -0.3;
		semaphore_post(&B);
		OS_waitNextPeriod();
	}
}
void setPWM()
{
	TIM2->CCR1 = (int)((pid + 0.61)*TIM2->ARR);
}

void TaskCTRL() {
    while (1) {
    	semaphore_wait(&C);
    	setPWM();
    	semaphore_post(&C);
        OS_waitNextPeriod();
    }
}

void TaskCH() {
    while (1) {
    	//chamada pelo interrupt, altera o setpoint
    	semaphore_wait(&D);
    	if(setPoint == 400) setPoint = 200;
    	else setPoint = 400;
    	semaphore_post(&D);
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
	semaphore_init(&A, 1);
	semaphore_init(&B, 1);
	semaphore_init(&C, 1);
	semaphore_init(&D, 1);

	// Configure VL53L0X
	// usart_init(USART1, rcc_get_clock() / 115200);
	while(!VL53L0X_init(&myTOFsensor));
	VL53L0X_setMeasurementTimingBudget(&myTOFsensor, 20e3); // 20 ms
	VL53L0X_startContinuous(&myTOFsensor, 0);

    srand(time(NULL));

    OS_init();

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


