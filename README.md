# Total Bandwidth Server and Non-Preemptive Protocol Implementation on MirOS

## Student Information

Name: Italo Miranda Kusmin Alves  
Matriculation: 22101930

## Total Bandwidth Server Implementation

The total bandwidth server implementation was achieved by using the function `OS_TBSserver`, which utilizes the Total Bandwidth Server (TBS) algorithm to set the deadlines of tasks. The deadline is given by:

$$
D_i = \max(t_{current}, D_{last}) + \frac{C_i}{U_s}
$$

where \(D_i\) is the deadline, \(t_{current}\) is the current time, \(D_{last}\) is the last deadline, \(C_i\) is the computation time, and \(U_s\) is the server utilization.

The EDF (Earliest Deadline First) scheduler selects the task with the earliest deadline for execution. Several modifications were made to enable the OS to schedule aperiodic tasks, such as altering the OS thread structure to include parameters for aperiodic and periodic tasks and implementing the `OSAperiodic_thread_start` function to start and queue aperiodic tasks.

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
