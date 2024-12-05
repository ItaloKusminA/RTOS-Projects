# PID Controller for the Height of a Ping Pong Ball on MiROS

## Students Information

Names: Gustavo Moro, Italo Miranda Kusmin Alves, Pedro Augusto Dantas Vargas
Matriculations: 22101929, 22101930, 22103666.

## Utilized components

## Periodic Tasks

## Aperiodic Tasks

## Critical regions

## Schedulability Analysis

The schedulability analysis ensures that all tasks meet their deadlines, both periodic and aperiodic. The computation time was measured in the second project. Then, a significant safety margin was incorporated into the execution times to guarantee that tasks complete within their deadlines even under varying load conditions. This is crucial because the computation time of each task is needed by the TBS to calculate the deadlines.

### Task Execution Times, Deadlines, and Periods

| Task Name | Execution Time (ms) | Deadline (ms) | Period (ms) |
|-----------|----------------------|---------------|-------------|
| SENS      | 50                   | 200           | 200         |
| CTRL      | 100                  | 500           | 500         |

### Aperiodic Tasks

| Task Name | Execution Time (ms) | Deadline (ms) |
|-----------|----------------------|---------------|
| DIAG      | 30                   | -             |

### Utilization Calculation

To determine if the tasks are schedulable using the Earliest Deadline First (EDF) algorithm, we need to ensure that the total utilization is less than or equal to 1. The utilization $U$ for each periodic task is calculated as follows:

$$ U_i = \frac{C_i}{T_i} $$

where:
- $C_i$ is the execution time of the task.
- $T_i$ is the period of the task.

#### Task SENS
$$ U_{SENS} = \frac{50 \, \text{ms}}{200 \, \text{ms}} = 0.25 $$

#### Task CTRL
$$ U_{CTRL} = \frac{100 \, \text{ms}}{500 \, \text{ms}} = 0.20 $$

### Total Utilization for Periodic Tasks
$$ U_{total} = U_{SENS} + U_{CTRL} = 0.25 + 0.20 = 0.45 $$

Since the total utilization for periodic tasks is 0.45, which is less than 1, these periodic tasks are schedulable under the EDF algorithm.

### Schedulability of Aperiodic Task 

For aperiodic tasks, schedulability depends on the available slack time in the schedule of periodic tasks. The task `DIAG` has an execution time of 30 ms. Since the utilization of the periodic tasks is 0.45, there is remaining utilization available for aperiodic tasks:

$$ U_{remaining} = 1 - U_{total} = 1 - 0.45 = 0.55 $$

To determine if `DIAG` can be scheduled, we need to ensure that it can fit within the available slack time. Given that it does not have a specified period and is event-driven (triggered by an interrupt on PB0), we check if the system can accommodate its execution time within the slack.

- Execution Time of DIAG: 30 ms
- Total available time in a 200 ms period (least common multiple of periods of periodic tasks): $200 \times 0.55 = 110 \ \text{ms}$

Since the execution time of `DIAG` (30 ms) is less than the available slack time (110 ms) in a 200 ms window, `DIAG` is schedulable within the system.

### Conclusion

The given periodic tasks (SENS and CTRL) are schedulable under the EDF algorithm as their total utilization is less than 1. The aperiodic task (DIAG) is also schedulable as its execution time fits within the available slack time of the periodic tasks.
