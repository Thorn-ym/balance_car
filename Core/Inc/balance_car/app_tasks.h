#ifndef BALANCE_CAR_APP_TASKS_H
#define BALANCE_CAR_APP_TASKS_H

#include "stm32f1xx_hal.h"
#include "balance_car/app_sensors.h"
#include "balance_car/balance_control.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"
#include <stdint.h>

typedef enum {
    APP_CMD_SOURCE_REMOTE = 1,
    APP_CMD_SOURCE_RASPI = 2,
    APP_CMD_SOURCE_OZONE = 3
} AppCommandSource_t;

typedef struct {
    uint8_t source;
    uint8_t run_valid;
    uint8_t run_enable;
    uint8_t reset_pid;
    uint8_t clear_fault;
    float speed_target;
    float turn_target;
} AppControlCommand_t;

#define APP_REMOTE_LINE_MAX 31U
#define APP_RASPI_LINE_MAX  79U

typedef enum {
    APP_PID_TARGET_SPEED = 1,
    APP_PID_TARGET_ANGLE = 2,
    APP_PID_TARGET_TURN = 3
} AppPidTarget_t;

typedef struct {
    uint8_t source;
    uint8_t target;
    float kp;
    float ki;
    float kd;
} AppPidUpdate_t;

typedef struct {
    PID_t speed;
    PID_t angle;
    PID_t turn;
} AppPidSnapshot_t;

typedef struct {
    char text[APP_REMOTE_LINE_MAX + 1U];
} AppRemoteLine_t;

typedef struct {
    char text[APP_RASPI_LINE_MAX + 1U];
} AppRaspiLine_t;

typedef struct {
    uint32_t balance_wake_count;
    uint32_t balance_timeout_count;
    uint32_t balance_last_ms;
    uint32_t raspi_loop_count;
    uint32_t raspi_last_ms;
    uint32_t bluetooth_loop_count;
    uint32_t bluetooth_last_ms;
    uint32_t sensor_loop_count;
    uint32_t sensor_last_ms;
} AppTaskMonitor_t;

extern SemaphoreHandle_t Sem_BalanceControl;
extern SemaphoreHandle_t Sem_BalanceStateMutex;
extern SemaphoreHandle_t Sem_SensorStateMutex;
extern SemaphoreHandle_t Sem_BluetoothTxMutex;
extern SemaphoreHandle_t Sem_RaspberryTxMutex;
extern SemaphoreHandle_t Sem_BluetoothRxReady;
extern QueueHandle_t Queue_ControlCommand;
extern QueueHandle_t Queue_PidUpdate;
extern QueueHandle_t Queue_BluetoothRxLine;
extern QueueHandle_t Queue_RaspberryRxLine;
extern volatile AppTaskMonitor_t g_app_task_monitor;

HAL_StatusTypeDef AppTasks_Init(void);
void AppTasks_NotifyBalanceFromISR(void);
void AppTasks_NotifyBluetoothRxFromISR(BaseType_t *higher_priority_task_woken);
void AppTasks_SubmitControlCommandFromTask(const AppControlCommand_t *command);
void AppTasks_SubmitPidUpdateFromTask(const AppPidUpdate_t *update);
void AppTasks_CopyBalanceState(BalanceCarState_t *state);
void AppTasks_CopySensorState(AppSensorState_t *state);
void AppTasks_CopyPidSnapshot(AppPidSnapshot_t *snapshot);

void Task_BalanceControl(void const *argument);
void Task_UartRaspberry(void const *argument);
void Task_BluetoothApp(void const *argument);
void Task_SensorCollect(void const *argument);

#endif
