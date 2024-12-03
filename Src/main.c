#include <stdint.h>
#include <stdlib.h>
#include <time.h>
#include "VL53L0X.h"
#include "miros.h"
#include "stm32f1xx_hal.h"

//em milis
#define CTRL_PERIOD 100
#define CTRL_DEADLINE 100
#define CTRL_WCCT 50

#define PID_PERIOD 50
#define PID_DEADLINE 50
#define PID_WCCT 10

#define CH_WCCT 30

static struct VL53L0X myTOFsensor = {.io_2v8 = true, .address = 0b0101001, .io_timeout = 500, .did_timeout = false};
uint16_t measuredValue = 0;
TIM_HandleTypeDef htim3;


typedef struct{
    OSThread taskT;
    uint32_t stackT[40];
} tasks_t;

periodicParameters CTRLPP;
aperiodicParameters CTRLAP;
periodicParameters PIDPP;
aperiodicParameters PIDAP;
periodicParameters CHPP;
aperiodicParameters CHAP;

tasks_t taskPID;
tasks_t taskCTRL;
tasks_t taskCH;

int setPoint = 500; //mm
int error = 0;
double errorIntegral = 0.0;
double errorDerivative = 0.0;
double previousError = 0.0;
uint32_t currentTick = 0;
const double Kp = -0.0001;
const double Ki = -0.00001;
const double Kd = -0.00001;
const double engineeringAdjust = 0.61; // % de dutyCycle
double pid = 0.0;
double dutyCycle = 0.0;


void TaskPID() {
    while (1) {
    	//Ler o sensor com I2C
    	measuredValue = VL53L0X_readRangeContinuousMillimeters(&myTOFsensor);
    	//Calcula o erro
    	uint32_t dt = ((HAL_GetTick() - currentTick)/1000);
    	error = setPoint - measuredValue;
    	errorIntegral += error*dt;
    	errorDerivative = (error - previousError)/dt;
    	previousError = error;
    	currentTick = HAL_GetTick();
    	//Calcula PID
    	pid = Kp*error + Ki*errorIntegral + Kd*errorDerivative;
        OS_waitNextPeriod();
    }
}

void setPWM(TIM_HandleTypeDef timer, uint32_t channel, uint16_t period, float dutyCycle)
{
    HAL_TIM_PWM_Stop(&timer, channel);          // stop generation of pwm
    TIM_OC_InitTypeDef sConfigOC;
    timer.Init.Period = period;                 // set the period duration
    HAL_TIM_PWM_Init(&timer);                   // reinitialise with new period value

    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = (uint16_t)(dutyCycle * period);  // set the pulse duration based on duty cycle
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    HAL_TIM_PWM_ConfigChannel(&timer, &sConfigOC, channel);

    HAL_TIM_PWM_Start(&timer, channel);         // start pwm generation
}

void TaskCTRL() {
    while (1) {
    	//Altera o PWM baseado no Kp, Ki e Kd
    	setPWM(htim3, TIM_CHANNEL_1, 1000, (engineeringAdjust + pid));
        OS_waitNextPeriod();
    }
}

void TaskCH() {
    while (1) {
    	//chamada pelo interrupt, altera o setpoint
    	setPoint = 300;
        OS_waitNextOccurence();
    }
}
uint32_t previousTick = 0;

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	uint32_t currentTick = HAL_GetTick();
	if (GPIO_Pin == GPIO_PIN_0 && (currentTick - previousTick) > 20){
		previousTick = currentTick;
		OS_TBS(&taskCH.taskT);
	}
}

static void MX_GPIO_Init(void);
static void MX_TIM3_Init(void);

int main() {
	HAL_Init();
	MX_GPIO_Init();
	MX_TIM3_Init();

	// Configure VL53L0X
	// usart_init(USART1, rcc_get_clock() / 115200);
	VL53L0X_init(&myTOFsensor);
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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin : PA6 (TIM3_CHANNEL_1) */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0U, 0U);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);
}

/* TIM1 init function */
/* TIM1 init function */
void MX_TIM3_Init(void)
{
    TIM_MasterConfigTypeDef sMasterConfig;
    TIM_OC_InitTypeDef sConfigOC;

    htim3.Instance = TIM3;
    htim3.Init.Prescaler = 0;
    htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim3.Init.Period = 0;
    htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    HAL_TIM_PWM_Init(&htim3);

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig);

    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1);
}



