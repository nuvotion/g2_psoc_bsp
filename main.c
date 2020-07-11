#include <project.h>

#include "print.h"

static int usb_up;

static void usb_poll(void) {
    if (USBFS_GetConfiguration()) {
        if (!usb_up) USBFS_CDC_Init();
        usb_up = 1;
    } else {
        usb_up = 0;
    }
}

void print(char *string) {
    if (!usb_up) return;
    while (!USBFS_CDCIsReady()) {}
    USBFS_PutData((uint8_t *)string, strlen(string));
}

static uint8_t sdlc_data[] = {
    0x15, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
    0x08, 0x09, 0x10, 0x11, 0x12, 0x13, 0x14, 0x15,
    0x16, 0x17, 0x18, 0x19, 0x20, 0x21, 0x22, 0x23,
    0x24, 0x25, 0x26, 0x27, 0x28, 0x29, 0x30, 0x31
};

CY_ISR(sys_tick_handler) {
    SDLC_Send(21, sdlc_data);
}

int main(void) {
    int i = 0;
    int blink = 0;

    USBFS_Start(0, USBFS_DWR_VDDD_OPERATION);
    SDLC_Setup();
    SYS_TICK_Start();
    SYS_TICK_IRQ_StartEx(sys_tick_handler);

    LED_0_Write(1);
    LED_1_Write(1);

    CyGlobalIntEnable;

    for(;;) {
        if (i == 200000) {
            LED_0_Write(blink);
            blink = !blink;
            i = 0;
        }

        usb_poll();
        i++;
    }
}
