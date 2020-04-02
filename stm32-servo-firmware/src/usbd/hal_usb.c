#include "stm32xxxx.h"
#include "xpd_usb.h"
#include "usb_device.h"

extern USBD_HandleType gUsbDevice;


void HAL_USB_OTG_FS_MspInit(void* handle)
{
    // GPIO setup apparently not needed on STM32G4
    // GPIO_InitTypeDef GPIO_InitStruct = {
    //     .Pin = GPIO_PIN_11 | GPIO_PIN_12,
    //     .Mode = GPIO_MODE_AF_PP,
    //     .Pull = GPIO_NOPULL,
    //     .Speed = GPIO_SPEED_FREQ_VERY_HIGH,
    //     .Alternate = GPIO_AF10_OTG_FS,
    // };
    // HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**USB GPIO Configuration    
    PA11     ------> USB_DM
    PA12     ------> USB_DP 
    */
    GPIO_InitTypeDef GPIO_InitStruct = {
        .Pin    = GPIO_PIN_11 | GPIO_PIN_12,
        .Mode   = GPIO_MODE_ANALOG,
        .Pull   = GPIO_NOPULL,
    };
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    __HAL_RCC_USB_CLK_ENABLE();

    HAL_NVIC_SetPriority(USB_HP_IRQn, 2, 0);
    HAL_NVIC_SetPriority(USB_LP_IRQn, 3, 0);
    HAL_NVIC_EnableIRQ(USB_HP_IRQn);
    HAL_NVIC_EnableIRQ(USB_LP_IRQn);
}

void HAL_USB_OTG_FS_MspDeInit(void* handle)
{
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_11 | GPIO_PIN_12);

    HAL_NVIC_DisableIRQ(USB_LP_IRQn);
    HAL_NVIC_DisableIRQ(USB_HP_IRQn);

    __HAL_RCC_USB_CLK_DISABLE();
}

void HAL_USBD_Setup(void)
{
    USBD_HandleType* const self = &gUsbDevice;

    USB_INST2HANDLE(self, USB);
    self->Callbacks.DepInit = HAL_USB_OTG_FS_MspInit;
    self->Callbacks.DepDeinit = HAL_USB_OTG_FS_MspDeInit;
}

void USB_HP_IRQHandler(void)
{
    USB_vIRQHandler(&gUsbDevice);
}

void USB_LP_IRQHandler(void)
{
    USB_vIRQHandler(&gUsbDevice);
}
