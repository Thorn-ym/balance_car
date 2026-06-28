#include "balance_car/app_tasks.h"

#include "balance_car/app_sensors.h"
#include "balance_car/balance_control.h"
#include "balance_car/raspi_link.h"
#include "balance_car/remote_control.h"
#include "cmsis_os.h"
#include "task.h"

#define TASK_BALANCE_STACK_WORDS      384U
#define TASK_RASPI_STACK_WORDS        512U
#define TASK_BLUETOOTH_STACK_WORDS    512U
#define TASK_SENSOR_STACK_WORDS       256U
#define QUEUE_CONTROL_DEPTH           8U
#define QUEUE_PID_DEPTH               8U
#define QUEUE_REMOTE_RX_DEPTH         8U
#define QUEUE_RASPI_RX_DEPTH          1U
#define SENSOR_IDLE_PERIOD_MS         500U
#define SENSOR_SERVICE_PERIOD_MS      20U

SemaphoreHandle_t Sem_BalanceControl;
SemaphoreHandle_t Sem_BalanceStateMutex;
SemaphoreHandle_t Sem_SensorStateMutex;
SemaphoreHandle_t Sem_BluetoothTxMutex;
SemaphoreHandle_t Sem_RaspberryTxMutex;
SemaphoreHandle_t Sem_BluetoothRxReady;
QueueHandle_t Queue_ControlCommand;
QueueHandle_t Queue_PidUpdate;
QueueHandle_t Queue_BluetoothRxLine;
QueueHandle_t Queue_RaspberryRxLine;
volatile AppTaskMonitor_t g_app_task_monitor;

static BalanceCarState_t s_balance_snapshot;
static AppSensorState_t s_sensor_snapshot;
static AppPidSnapshot_t s_pid_snapshot;

static void AppTasks_ApplyQueuedCommands(void)
{
    AppControlCommand_t command;

    while (Queue_ControlCommand != NULL &&
           xQueueReceive(Queue_ControlCommand, &command, 0U) == pdPASS) {
        BalanceCar_ApplyControlCommand(command.run_valid,
                                       command.run_enable,
                                       command.reset_pid,
                                       command.clear_fault,
                                       command.speed_target,
                                       command.turn_target);
    }
}

static void AppTasks_ApplyQueuedPidUpdates(void)
{
    AppPidUpdate_t update;
    PID_t *pid = NULL;

    while (Queue_PidUpdate != NULL &&
           xQueueReceive(Queue_PidUpdate, &update, 0U) == pdPASS) {
        if (update.target == APP_PID_TARGET_SPEED) {
            pid = &g_speed_pid;
        } else if (update.target == APP_PID_TARGET_ANGLE) {
            pid = &g_angle_pid;
        } else if (update.target == APP_PID_TARGET_TURN) {
            pid = &g_turn_pid;
        } else {
            pid = NULL;
        }

        if (pid != NULL) {
            pid->Kp = update.kp;
            pid->Ki = update.ki;
            pid->Kd = update.kd;
        }
    }
}

HAL_StatusTypeDef AppTasks_Init(void)
{
    Sem_BalanceControl = xSemaphoreCreateBinary();
    Sem_BalanceStateMutex = xSemaphoreCreateMutex();
    Sem_SensorStateMutex = xSemaphoreCreateMutex();
    Sem_BluetoothTxMutex = xSemaphoreCreateMutex();
    Sem_RaspberryTxMutex = xSemaphoreCreateMutex();
    Sem_BluetoothRxReady = xSemaphoreCreateBinary();
    Queue_ControlCommand = xQueueCreate(QUEUE_CONTROL_DEPTH, sizeof(AppControlCommand_t));
    Queue_PidUpdate = xQueueCreate(QUEUE_PID_DEPTH, sizeof(AppPidUpdate_t));
    Queue_BluetoothRxLine = xQueueCreate(QUEUE_REMOTE_RX_DEPTH, sizeof(AppRemoteLine_t));
    Queue_RaspberryRxLine = xQueueCreate(QUEUE_RASPI_RX_DEPTH, sizeof(AppRaspiLine_t));

    if (Sem_BalanceControl == NULL ||
        Sem_BalanceStateMutex == NULL ||
        Sem_SensorStateMutex == NULL ||
        Sem_BluetoothTxMutex == NULL ||
        Sem_RaspberryTxMutex == NULL ||
        Sem_BluetoothRxReady == NULL ||
        Queue_ControlCommand == NULL ||
        Queue_PidUpdate == NULL ||
        Queue_BluetoothRxLine == NULL ||
        Queue_RaspberryRxLine == NULL) {
        return HAL_ERROR;
    }

    osThreadDef(Task_BalanceControl, Task_BalanceControl, osPriorityRealtime, 0, TASK_BALANCE_STACK_WORDS);
    osThreadDef(Task_UartRaspberry, Task_UartRaspberry, osPriorityHigh, 0, TASK_RASPI_STACK_WORDS);
    osThreadDef(Task_BluetoothApp, Task_BluetoothApp, osPriorityNormal, 0, TASK_BLUETOOTH_STACK_WORDS);
    osThreadDef(Task_SensorCollect, Task_SensorCollect, osPriorityLow, 0, TASK_SENSOR_STACK_WORDS);

    if (osThreadCreate(osThread(Task_BalanceControl), NULL) == NULL ||
        osThreadCreate(osThread(Task_UartRaspberry), NULL) == NULL ||
        osThreadCreate(osThread(Task_BluetoothApp), NULL) == NULL ||
        osThreadCreate(osThread(Task_SensorCollect), NULL) == NULL) {
        return HAL_ERROR;
    }

    return HAL_OK;
}

void AppTasks_NotifyBalanceFromISR(void)
{
    BaseType_t higher_priority_task_woken = pdFALSE;

    if (Sem_BalanceControl == NULL) {
        return;
    }
    (void)xSemaphoreGiveFromISR(Sem_BalanceControl, &higher_priority_task_woken);
    portYIELD_FROM_ISR(higher_priority_task_woken);
}

void AppTasks_NotifyBluetoothRxFromISR(BaseType_t *higher_priority_task_woken)
{
    if (Sem_BluetoothRxReady == NULL || higher_priority_task_woken == NULL) {
        return;
    }
    (void)xSemaphoreGiveFromISR(Sem_BluetoothRxReady, higher_priority_task_woken);
}

void AppTasks_SubmitControlCommandFromTask(const AppControlCommand_t *command)
{
    AppControlCommand_t dropped;

    if (command == NULL || Queue_ControlCommand == NULL) {
        return;
    }
    if (xTaskGetSchedulerState() == taskSCHEDULER_NOT_STARTED) {
        BalanceCar_ApplyControlCommand(command->run_valid,
                                       command->run_enable,
                                       command->reset_pid,
                                       command->clear_fault,
                                       command->speed_target,
                                       command->turn_target);
        return;
    }
    if (xQueueSendToBack(Queue_ControlCommand, command, 0U) != pdPASS) {
        (void)xQueueReceive(Queue_ControlCommand, &dropped, 0U);
        (void)xQueueSendToBack(Queue_ControlCommand, command, 0U);
    }
}

void AppTasks_SubmitPidUpdateFromTask(const AppPidUpdate_t *update)
{
    AppPidUpdate_t dropped;

    if (update == NULL || Queue_PidUpdate == NULL) {
        return;
    }
    if (xTaskGetSchedulerState() == taskSCHEDULER_NOT_STARTED) {
        if (update->target == APP_PID_TARGET_SPEED) {
            g_speed_pid.Kp = update->kp;
            g_speed_pid.Ki = update->ki;
            g_speed_pid.Kd = update->kd;
        } else if (update->target == APP_PID_TARGET_ANGLE) {
            g_angle_pid.Kp = update->kp;
            g_angle_pid.Ki = update->ki;
            g_angle_pid.Kd = update->kd;
        } else if (update->target == APP_PID_TARGET_TURN) {
            g_turn_pid.Kp = update->kp;
            g_turn_pid.Ki = update->ki;
            g_turn_pid.Kd = update->kd;
        }
        return;
    }
    if (xQueueSendToBack(Queue_PidUpdate, update, 0U) != pdPASS) {
        (void)xQueueReceive(Queue_PidUpdate, &dropped, 0U);
        (void)xQueueSendToBack(Queue_PidUpdate, update, 0U);
    }
}

void AppTasks_CopyBalanceState(BalanceCarState_t *state)
{
    if (state == NULL) {
        return;
    }
    if (Sem_BalanceStateMutex != NULL &&
        xSemaphoreTake(Sem_BalanceStateMutex, pdMS_TO_TICKS(2U)) == pdPASS) {
        taskENTER_CRITICAL();
        s_balance_snapshot = g_balance_state;
        taskEXIT_CRITICAL();
        *state = s_balance_snapshot;
        (void)xSemaphoreGive(Sem_BalanceStateMutex);
    } else {
        *state = s_balance_snapshot;
    }
}

void AppTasks_CopySensorState(AppSensorState_t *state)
{
    if (state == NULL) {
        return;
    }
    if (Sem_SensorStateMutex != NULL &&
        xSemaphoreTake(Sem_SensorStateMutex, pdMS_TO_TICKS(2U)) == pdPASS) {
        s_sensor_snapshot = g_sensor_state;
        *state = s_sensor_snapshot;
        (void)xSemaphoreGive(Sem_SensorStateMutex);
    } else {
        *state = s_sensor_snapshot;
    }
}

void AppTasks_CopyPidSnapshot(AppPidSnapshot_t *snapshot)
{
    if (snapshot == NULL) {
        return;
    }
    if (Sem_BalanceStateMutex != NULL &&
        xSemaphoreTake(Sem_BalanceStateMutex, pdMS_TO_TICKS(2U)) == pdPASS) {
        taskENTER_CRITICAL();
        s_pid_snapshot.speed = g_speed_pid;
        s_pid_snapshot.angle = g_angle_pid;
        s_pid_snapshot.turn = g_turn_pid;
        taskEXIT_CRITICAL();
        *snapshot = s_pid_snapshot;
        (void)xSemaphoreGive(Sem_BalanceStateMutex);
    } else {
        *snapshot = s_pid_snapshot;
    }
}

void Task_BalanceControl(void const *argument)
{
    (void)argument;

    for (;;) {
        if (xSemaphoreTake(Sem_BalanceControl, pdMS_TO_TICKS(30U)) == pdPASS) {
            g_app_task_monitor.balance_wake_count++;
            g_app_task_monitor.balance_last_ms = HAL_GetTick();
            if (Sem_BalanceStateMutex != NULL &&
                xSemaphoreTake(Sem_BalanceStateMutex, pdMS_TO_TICKS(1U)) == pdPASS) {
                AppTasks_ApplyQueuedCommands();
                AppTasks_ApplyQueuedPidUpdates();
                BalanceCar_ControlStep10ms();
                (void)xSemaphoreGive(Sem_BalanceStateMutex);
            } else {
                AppTasks_ApplyQueuedCommands();
                AppTasks_ApplyQueuedPidUpdates();
                BalanceCar_ControlStep10ms();
            }
        } else {
            g_app_task_monitor.balance_timeout_count++;
            g_app_task_monitor.balance_last_ms = HAL_GetTick();
            BalanceCar_ApplyControlCommand(1U, 0U, 0U, 0U, 0.0f, 0.0f);
            BalanceCar_ServiceRequests();
        }
    }
}

void Task_UartRaspberry(void const *argument)
{
    TickType_t last_wake;

    (void)argument;
    last_wake = xTaskGetTickCount();

    for (;;) {
        g_app_task_monitor.raspi_loop_count++;
        g_app_task_monitor.raspi_last_ms = HAL_GetTick();
        RaspiLink_Background();
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(20U));
    }
}

void Task_BluetoothApp(void const *argument)
{
    (void)argument;

    for (;;) {
        g_app_task_monitor.bluetooth_loop_count++;
        g_app_task_monitor.bluetooth_last_ms = HAL_GetTick();
        RemoteControl_Background();
        (void)xSemaphoreTake(Sem_BluetoothRxReady, pdMS_TO_TICKS(50U));
    }
}

void Task_SensorCollect(void const *argument)
{
    TickType_t last_wake;
    TickType_t delay_ticks;

    (void)argument;
    last_wake = xTaskGetTickCount();

    for (;;) {
        g_app_task_monitor.sensor_loop_count++;
        g_app_task_monitor.sensor_last_ms = HAL_GetTick();
        if (Sem_SensorStateMutex != NULL &&
            xSemaphoreTake(Sem_SensorStateMutex, pdMS_TO_TICKS(5U)) == pdPASS) {
            AppSensors_Background();
            (void)xSemaphoreGive(Sem_SensorStateMutex);
        } else {
            AppSensors_Background();
        }
        delay_ticks = AppSensors_NeedsServiceSoon() != 0U ?
                      pdMS_TO_TICKS(SENSOR_SERVICE_PERIOD_MS) :
                      pdMS_TO_TICKS(SENSOR_IDLE_PERIOD_MS);
        vTaskDelayUntil(&last_wake, delay_ticks);
    }
}

void vApplicationMallocFailedHook(void)
{
    taskDISABLE_INTERRUPTS();
    for (;;) {
    }
}

void vApplicationStackOverflowHook(TaskHandle_t task, char *name)
{
    (void)task;
    (void)name;
    taskDISABLE_INTERRUPTS();
    for (;;) {
    }
}
