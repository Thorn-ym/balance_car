#include "balance_car/raspi_link.h"
#include "balance_car/remote_control.h"

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (RemoteControl_RxCpltCallback(huart) != 0U) {
        return;
    }
    (void)RaspiLink_RxCpltCallback(huart);
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (RemoteControl_TxCpltCallback(huart) != 0U) {
        return;
    }
    (void)RaspiLink_TxCpltCallback(huart);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (RemoteControl_ErrorCallback(huart) != 0U) {
        return;
    }
    (void)RaspiLink_ErrorCallback(huart);
}
