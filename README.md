# Total Bandwidth Server and Non-Preemptive Protocol Implementation on MirOS

## Student Information

Name: Italo Miranda Kusmin Alves  
Matriculation: 22101930

## Total Bandwidth Server Implementation

To implement the Total Bandwidth Server (TBS), the first change was to modify the `OSThread` structure by adding two additional parameters: `aperiodicParameters` for aperiodic tasks, which store the computation time of each task, and `periodicParameters`, which store the relative deadline and relative period. The period and deadline are dynamically adjusted to select the task with the earliest deadline.

A new vector of structs, `OS_APThread`, was created to store the started aperiodic tasks. Additionally, `OS_APreadyIndex` was introduced to keep track of the aperiodic tasks that are ready to run.

Next, the `OSAperiodic_thread_start` function was implemented. Its purpose is similar to `OSThread_start`, but with a key difference: aperiodic tasks start with the ready index set to 0 (not ready to run). This function initializes the `aperiodicParameters` structure with the received computation time and sets the deadline to `UINT32_MAX`. In contrast, `OSThread_start` initializes the `periodicParameters` with the user-provided parameters.

The TBS implementation is achieved using the `OS_TBSserver` function, which applies the TBS algorithm to set task deadlines and update the ready index to 1 (ready to run). The deadline is calculated as follows:

$$
D_i = \max(t_{current}, D_{last}) + \frac{C_i}{U_s}
$$

where \ D_i \ is the deadline, \ t_{current} \ is the current time, \ D_{last} \ is the last deadline, \ C_i \ is the computation time, and \ U_s \ is the server utilization. Essentially, the deadline assigned to the task is the sum of the current time (or the last deadline, whichever is greater) and the ratio of the task's computation time to the server utilization. The Earliest Deadline Scheduler then schedules the aperiodic task if its deadline is earlier than those of the periodic tasks.

Finally, the `OS_waitNextOccurence` function was implemented, functioning similarly to `OS_waitNextPeriod`. It handles the end of a task's execution by resetting the deadline to `UINT32_MAX` and the ready index to 0 (not ready to run).


## Aperiodic Task Implementation

An aperiodic task was implemented by converting a previously periodic function into an aperiodic task. This task is triggered every time the button PB0 is pressed. The implementation involves calculating the server utilization and defining the task deadline dynamically.

## Non-Preemptive Protocol Implementation

The non-preemptive protocol was implemented by adding a parameter `NPPprio` to the `OS_thread` structure. This parameter is set to the maximum value when a thread enters a critical region, preventing other threads from being scheduled. The thread keeps track of the critical regions it enters, ensuring that `NPPprio` is reset only when it exits the last critical region.

## Schedulability Analysis

The schedulability analysis ensures that all tasks meet their deadlines, both periodic and aperiodic. A significant safety margin was incorporated into the execution times to guarantee that tasks complete within their deadlines even under varying load conditions.

### Task Execution Times, Deadlines, and Periods

| Task Name     | Execution Time (ms) | Deadline (ms) | Period (ms) |
|---------------|----------------------|---------------|-------------|
| SENS          | 50                   | 200           | 200         |
| CTRL          | 100                  | 500           | 500         |

### Aperiodic Tasks

| Task Name       | Execution Time (ms) | Deadline (ms) |
|-----------------|----------------------|---------------|
| DIAG          | 30                   | -             | -           |

### Comments on Safety Margins

To ensure the reliability of the system, a significant safety margin was added to the execution times of the tasks. This safety margin accounts for unexpected delays and variations in task execution, ensuring that all tasks meet their deadlines even under high load or unpredictable conditions.

## System Schedulability with EDF and TBS Server

### System Description

The system is based on the Minimal Real-time Operating System (MiROS) and utilizes the Earliest Deadline First (EDF) scheduling algorithm for periodic tasks and a Total Bandwidth Server (TBS) for aperiodic tasks. The combination of EDF with the TBS server ensures that all periodic tasks are completed within their deadlines while efficiently managing the execution of aperiodic tasks.

### Periodic Tasks

The periodic tasks in the system are defined with their periods, deadlines, and worst-case computation times (WCCT).

## Problemns solved with the STM32 BluePill

Many times, with the STM32, its   
