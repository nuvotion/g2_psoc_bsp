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
    static char print_string[64];
    if (!usb_up) return;
    strcpy(print_string, string);
    while (!USBFS_CDCIsReady()) {}
    USBFS_PutData((uint8_t *)print_string, strlen(string));
}

static uint8_t wr_data[6] = { 0x04, 0x00, 0x56, 0x78, 0x9A, 0xBC };
//static uint8_t wr_data[6] = { 0 };

CY_ISR(sys_tick_handler) {
    /* Prepare write buffer */
    I2C_UFM_Write(0x10, 6, wr_data);

    wr_data[4]++;
}

int main(void) {
    int i = 0;
    int blink = 0;

    USBFS_Start(0, USBFS_DWR_VDDD_OPERATION);

    CySysTickStart();
    CySysTickSetReload(BCLK__BUS_CLK__HZ/6000);
    CySysTickClear();
    CySysTickSetCallback(0, sys_tick_handler);
    
    I2C_UFM_Setup();
    LOCAL_OUTPUT_Write(0x7);

    CyGlobalIntEnable;

    for(;;) {
        if (i == 1000000) {
            LOCAL_OUTPUT_Write(blink ? 0x5 : 0x07);
            print("Loop running\n");
            blink = !blink;
            i = 0;
        }

        usb_poll();
        i++;
    }
}
