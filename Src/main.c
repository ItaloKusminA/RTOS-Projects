#include <stdint.h>
#include <stdlib.h>
#include <time.h>
#include "miros.h"

#define N_PRODUCERS 2
#define N_CONSUMERS 2

typedef struct {
    OSThread producerT;
    uint32_t stackP[40];
} productor_t;

typedef struct {
    OSThread consumerT;
    uint32_t stackC[40];
} consumer_t;

productor_t producers[N_PRODUCERS];
consumer_t consumers[N_CONSUMERS];
semaphore_t empty;
semaphore_t full;
semaphore_t mutex;
const uint8_t emptyStart = 200;
const uint8_t fullStart = 0;
uint8_t buffer = 0;

uint8_t delay_generator(){
	return ((rand() % 100) + 1);
}

void producer() {
    while (1) {
    	semaphore_wait(&empty);
        OS_delay(delay_generator());
        semaphore_wait(&mutex);
        buffer++;
        semaphore_post(&mutex);
        semaphore_post(&full);
    }
}

void consumer() {
    while (1) {
    	semaphore_wait(&full);
        OS_delay(delay_generator());
        semaphore_wait(&mutex);
        buffer--;
        semaphore_post(&mutex);
        semaphore_post(&empty);
    }
}

uint32_t stack_idleThread[40];

int main() {
    srand(time(NULL));

    semaphore_init(&empty, emptyStart);
    semaphore_init(&full, fullStart);
    semaphore_init(&mutex, 1);

    OS_init(stack_idleThread, sizeof(stack_idleThread));

    for (int i = 0; i < N_PRODUCERS; i++) {
        OSThread_start(&(producers[i].producerT),
        			   (2*i+1),
                       &producer,
                       producers[i].stackP, sizeof(producers[i].stackP));
    }

    for (int i = 0; i < N_CONSUMERS; i++) {
        OSThread_start(&(consumers[i].consumerT),
        			   (2*i+2),
                       &consumer,
                       consumers[i].stackC, sizeof(consumers[i].stackC));
    }

    OS_run();

    return 0;
}
