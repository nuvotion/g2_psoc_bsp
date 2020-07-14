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

static uint8_t sdlc_tx_data[] __attribute__((aligned(4))) = {
    0x15, 0x00, 0x00, 0x03, 0x04, 0x05, 0x06, 0x07,
    0x08, 0x09, 0x10, 0x11, 0x12, 0x13, 0x14, 0x15,
    0x16, 0x17, 0x18, 0x19, 0x20, 0x21, 0x22, 0x23,
    0x24, 0x25, 0x26, 0x27, 0x28, 0x29, 0x30, 0x31
};

static uint8_t sdlc_rx_data[8] __attribute__((aligned(4)));
static uint8_t sdlc_bad_data[8] __attribute__((aligned(4)));
static int rx_pkts, tx_pkts, rx_bytes;

CY_ISR(sys_tick_handler) {
    if (sdlc_rx_data[0] == 0x15) rx_pkts++;
    else {
        memcpy(sdlc_bad_data, sdlc_rx_data, 8);
        rx_bytes = SDLC_GetRxBytes();
    }
    sdlc_rx_data[0] = 0;
    tx_pkts++;
    sdlc_tx_data[4] = tx_pkts & 0xff;
    sdlc_tx_data[3] = (tx_pkts >> 8) * 0xff;
    SDLC_SendReceive(7, 7, sdlc_tx_data, sdlc_rx_data);
}

int main(void) {
    int i = 0;
    int blink = 0;
    int j;

    USBFS_Start(0, USBFS_DWR_VDDD_OPERATION);
    SDLC_Setup();
    SYS_TICK_Start();
    SYS_TICK_IRQ_StartEx(sys_tick_handler);

    LED_0_Write(1);
    LED_1_Write(1);

    CyGlobalIntEnable;

    for(;;) {
        if (i == 1000000) {
            LED_0_Write(blink);
            print("Bytes: ");
            print(print_num(rx_bytes));
            print(", data: ");
            for (j = 0; j < 8; j++) {
                print(print_hex(sdlc_bad_data[j]));
                print(", ");
            }
            print("dropped pkts: ");
            print(print_num(tx_pkts - rx_pkts));
            print(" out of ");
            print(print_num(tx_pkts));
            print("\n");
            blink = !blink;
            i = 0;
        }

        usb_poll();
        i++;
    }
}
