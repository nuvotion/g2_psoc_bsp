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

void send_sdlc(void) {
#define STATE_LENGTH_REG \
            (*(reg8 *) sdlc_1_tx_ctrl_dp_u0__A0_REG)
#define STATE_PREAMBLE_REG \
            (*(reg8 *) sdlc_1_tx_ctrl_dp_u0__F1_REG)
#define TX_DATA_REG \
            (*(reg16 *) sdlc_1_sdlc_dp_dp_u0__16BIT_F1_REG)

    STATE_LENGTH_REG = (7+2)*8-1;

    TX_DATA_REG = 0x0015;
    TX_DATA_REG = 0xFF00;
    TX_DATA_REG = 0x00FF;
    TX_DATA_REG = 0x00FF;
    
    STATE_PREAMBLE_REG = 0xFF;
    STATE_PREAMBLE_REG = 0xFF;
    STATE_PREAMBLE_REG = 0xFF;
    STATE_PREAMBLE_REG = 0x7E;
}

int main(void) {
    int i = 0;
    int blink = 0;

    USBFS_Start(0, USBFS_DWR_VDDD_OPERATION);

    CyGlobalIntEnable;

#define SDLC_DPLL_CONTROL_REG \
	    (*(reg8 *) sdlc_1_dpll_dco__CONTROL_AUX_CTL_REG)
#define SDLC_BIT_COUNT_REG \
            (*(reg8 *) sdlc_1_sdlc_dp_bit_counter__CONTROL_AUX_CTL_REG)
#define SDLC_ZERO_INS_COUNT_REG \
            (*(reg8 *) sdlc_1_sdlc_dp_zero_ins_counter__CONTROL_AUX_CTL_REG)
#define POLYNOMIAL_REG \
            (*(reg16 *) sdlc_1_sdlc_dp_dp_u0__16BIT_D0_REG)
#define CRC_MATCH_REG \
            (*(reg16 *) sdlc_1_sdlc_dp_dp_u0__16BIT_D1_REG)
#define STATE_CRC_REG \
            (*(reg8 *) sdlc_1_tx_ctrl_dp_u0__D0_REG)
#define STATE_SYNC_REG \
            (*(reg8 *) sdlc_1_tx_ctrl_dp_u0__D1_REG)

    SDLC_DPLL_CONTROL_REG   = 0x20;
    SDLC_BIT_COUNT_REG      = 0x20;
    SDLC_ZERO_INS_COUNT_REG = 0x20;
    POLYNOMIAL_REG          = 0x810;
    CRC_MATCH_REG           = 0xE2F0;
    STATE_CRC_REG           = 0x10;
    STATE_SYNC_REG          = 0x7E;

    LED_0_Write(1);
    LED_1_Write(1);

    for(;;) {
        if (i == 200000) {
            send_sdlc();
            LED_0_Write(blink);
            blink = !blink;
            i = 0;
        }

        usb_poll();
        i++;
    }
}
