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

int main(void) {
    USBFS_Start(0, USBFS_DWR_VDDD_OPERATION);

    CyGlobalIntEnable;

#define SDLC_DPLL_CONTROL_REG \
	    (*(reg8 *) sdlc_1_dpll_dco__CONTROL_AUX_CTL_REG)

    SDLC_DPLL_CONTROL_REG = 0x20;

    LED_0_Write(1);
    LED_1_Write(1);

    for(;;) {
        usb_poll();
    }
}
