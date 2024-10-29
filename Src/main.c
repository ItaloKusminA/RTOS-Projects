#include <stdint.h>
#include "miros.h"
#include "stm32f1xx_hal.h"

#define TASK1_DEADLINE 5
#define TASK1_PERIOD 8
#define TASK1_COMPUTATION_TIME 3
#define TASK2_DEADLINE 10
#define TASK2_PERIOD 14
#define TASK2_COMPUTATION_TIME 5


typedef struct {
    OSThread taskT;
    uint32_t stackT[40];
} tasks_t;

tasks_t task1;
tasks_t task2;
uint8_t current_task = 0;


void taskF() {
	while(1){
		current_task = 1;
	}
}
void taskS() {
	while(1){
		current_task = 2;
	}
}

int main() {

    OS_init();

    OSThread_start(
        &task1.taskT,
		TASK1_DEADLINE,
		TASK1_PERIOD,
		TASK1_COMPUTATION_TIME,
        &taskF,
        &task1.stackT, sizeof(task1.stackT));

    OSThread_start(
        &task2.taskT,
		TASK2_DEADLINE,
		TASK2_PERIOD,
		TASK2_COMPUTATION_TIME,
        &taskS,
        &task2.stackT, sizeof(task2.stackT));

    OS_run();

    return 0;
}
