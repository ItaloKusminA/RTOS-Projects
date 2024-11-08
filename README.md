# Periodic Task and EDF Scheduler Implementation on MirOS

## Student Information

Name: Italo Miranda Kusmin Alves  
Matriculation: 22101930

## EDF Scheduler Implementation
To implement the EDF scheduler, several changes were necessary, since it is a dynamic scheduler and the standard scheduler of MiROS is static. The main changes were:
- The `OSThread` structure gained some new parameters to assist the scheduler, which are: `ABSdeadline`, `ABSperiod`, `ABScomputation_time`, `deadline`, `period`, `computation_time`, and `index`. The "ABS" variables store the absolute value of the parameter, and the others store the dynamic value. The `index` stores the location of the thread in the `OS_thread` vector, which points to all the initialized threads. The idle thread always occupies the first index.
- A new vector called `OS_readyIndex` was created to indicate if the thread located in the `OS_thread` at the same index is ready to run or not, storing 1 for ready and 0 for not.
- `OS_sched` schedules the task with the index selected previously by the `OS_tick`. If there are no threads to be scheduled, the idle thread runs.
- `OS_run` runs the first loop to select the earliest deadline task, storing its index in a global variable used by `OS_sched`. It has to run the first loop because the OS has not had the first tick yet, and the scheduling function is called before the first tick.
- `OS_tick` always selects the earliest deadline task that has not been executed yet, taking care of the execution period of each task.
- `OS_delay` sets the `OS_readyIndex` to 0 when a task is delayed and sets the timeout of the delay. The timeout is decreased by the `OS_tick`. The use of this function can affect the schedulability of the system.
- `OSThread_start` was changed to set the task parameters in the structure, which are received from the user.

## Periodic Task Implementation
The periodic scheduler implementation is very simple:
- The tasks are declared in a structure that contains their threads and the stack to store them. They are then initialized by calling `OSThread_start` with the parameters `deadline`, `period`, and `computation_time`.
- The tasks run the `taskF` and `taskS` functions, which contain an infinite loop.
- The scheduler takes care of the rest.
