<<<<<<< HEAD
<<<<<<< HEAD
# Projects developed during the RTOS Discipline
## Project 1
Producer and consumer implementation using semaphores
## Project 2
EDF Scheduler implementation with periodic tasks 
=======
=======
>>>>>>> 00acecc (Changes in the names and prioritys)
# Producer and Consumer Implementation Using Semaphores on MirOS

## Student Information

Name: Italo Miranda Kusmin Alves  
Matriculation: 22101930

## Semaphore Implementation
The implementation of semaphores was done with a structure containing three functions:
- The structure contains the current value of the semaphore.
- The first function is `semaphore_init`, which assigns the chosen initialization value to the current value of the semaphore.
- The second function is `semaphore_wait`, which verifies if the semaphore's current value is equal to zero. If it is, a yield function is implemented to allow the OS to schedule the next thread until the semaphore's current value changes to a value greater than zero. Otherwise, the semaphore's current value is decreased.
- The third function is `semaphore_post`, which increases the semaphore's current value.

## Producer and Consumer Implementation
The producer and consumer implementation is based on "Modern Operating Systems" by Andrew S. Tanenbaum. It involves creating three semaphores: `empty`, `full`, and `mutex`, and two functions: `producer` and `consumer`.
- The `full` semaphore stores the number of occupied spaces in the buffer. When an item is produced, this semaphore is increased, and the `empty` semaphore is decreased. It should be initialized to 0 if the buffer is empty.
- The `empty` semaphore stores the number of unoccupied spaces in the buffer. When an item is consumed, this semaphore is increased, and the `full` semaphore is decreased. It should be initialized to the maximum value of the buffer if the buffer is empty.
- The `mutex` semaphore is a binary semaphore that protects the shared variable.

The `producer` function is initialized with a generic number of producer threads, which can be chosen in the defines at the beginning of the code. It takes a random time in the range from 1 to 100 ticks to produce an item, increasing the buffer value and the `full` semaphore while decreasing the `empty` semaphore. When the buffer is accessed, the mutex is locked.

The `consumer` function is initialized with a generic number of consumer threads, which can be chosen in the defines at the beginning of the code. It takes a random time in the range from 1 to 100 ticks to consume an item, increasing the `empty` semaphore value and decreasing the buffer and `full` semaphore values. When the buffer is accessed, the mutex is locked.

<<<<<<< HEAD
In this implementation, the producers have greater priority than the consumers.
>>>>>>> 3cf8092 (EDF scheduler)
=======
In this implementation, the producers and consumers have alternating priorities to ensure they are always scheduled alternately. Producers have odd priorities (1, 3, 5, ...) and consumers have even priorities (2, 4, 6, ...).

>>>>>>> 00acecc (Changes in the names and prioritys)
