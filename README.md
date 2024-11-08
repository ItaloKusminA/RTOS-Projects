# Periodic Task and EDF Scheduler Implementation on MirOS

## Student Information

Name: Italo Miranda Kusmin Alves  
Matriculation: 22101930

## EDF Scheduler Implementation
To implement the EDF scheduler, several changes were necessary, since it is a dynamic scheduler and the standard scheduler of MiROS is static. The main changes were:
- The `OSThread` structure gained some new parameters to assist the scheduler, which are: `ABSdeadline`, `ABSperiod`, `deadline`, `period`, and `index`. The "ABS" variables store the absolute value of the parameter, and the others store the dynamic value. The `index` stores the location of the thread in the `OS_thread` vector, which points to all the initialized threads. The idle thread always occupies the first index.
- A new vector called `OS_readyIndex` was created to indicate if the thread located in the `OS_thread` at the same index is ready to run or not, storing 1 for ready and 0 for not.
- `OS_sched` schedules the task with the index selected previously by the `OS_tick`. If there are no threads to be scheduled, the idle thread runs.
- `OS_run` runs the first loop to select the earliest deadline task, storing its index in a global variable used by `OS_sched`. It has to run the first loop because the OS has not had the first tick yet, and the scheduling function is called before the first tick.
- `OS_tick` always selects the earliest deadline task that has not been executed yet, taking care of the execution period of each task.
- `OS_delay` sets the `OS_readyIndex` to 0 when a task is delayed and sets the timeout of the delay. The timeout is decreased by the `OS_tick`. The use of this function can affect the schedulability of the system.
- `OSThread_start` was changed to set the task parameters in the structure, which are received from the user.
- `OS_waitNextPeriod` was implemented to be used in periodic tasks. When the task runs its application, it calls the function, which sets the `OS_readyIndex` to 0 and delays the task until the next period.
  
## Periodic Task Implementation
The periodic task implementation aims to abstract an injection system in an Engine Control Unit, with three main tasks:
- `TaskSensoring` aims to abstract the sensing of parameters of interest to `TaskControl` through the functions: `readCrankshaftPosition`, `readMassAirFlow`, `readEngineTemperature`, and `readOxygenSensor`. This task was implemented with a deadline of 2 and a period of 2.
- `TaskControl` aims to abstract the control of the injectors and ignition system through the functions: `calculateInjectionTime`, `calculateIgnitionAdvance`, `controlInjectors`, and `controlIgnitionSystem`. This task was implemented with a deadline of 5 and a period of 5.
- `TaskDiagnostics` aims to abstract the function of detecting system faults through the functions: `monitorSensors`, `monitorActuators`, and `detectFaults`. This task was implemented with a deadline of 500 and a period of 500.

To analyze the schedulability of these tasks, the computation time of each one was measured over 5 minutes, and the average computation time was calculated. To measure the time, `HAL_GetTick` was used to get the tick at the beginning and end of each task, and the average was updated.

## Example Table

| Task             | Computation Time | Deadline | Period |
|------------------|------------------|----------|--------|
| TaskSensoring    | 0.0058140927     | 2        | 2      |
| TaskControl      | 0.0271115694     | 5        | 5      |
| TaskDiagnostics  | 0.151287213      | 500      | 500    |

## Schedulability Analysis
To determine if the system is schedulable, we use the following formula for the utilization factor \(U\):

\[ U = \sum_{i=1}^{n} \frac{C_i}{T_i} \]

where \(C_i\) is the computation time and \(T_i\) is the period for each task \(i\).

Calculating the utilization factor for the given tasks:

\[ U = \frac{0.0058140927}{2} + \frac{0.0271115694}{5} + \frac{0.151287213}{500} \]

\[ U = 0.00290704635 + 0.00542231388 + 0.000302574426 \]

\[ U = 0.008631934656 \]

The utilization factor \(U = 0.008631934656\) is well below 1, indicating that the tasks are schedulable under the given deadlines and periods.

  
