#include <project.h>

static volatile uint8  `$INSTANCE_NAME`_mstrWrBufIndex;
static volatile uint8 *`$INSTANCE_NAME`_mstrWrBufPtr;
static volatile uint8  `$INSTANCE_NAME`_mstrWrBufSize;

CY_ISR(`$INSTANCE_NAME`_ISR) {
    if (`$INSTANCE_NAME`_mstrWrBufIndex < `$INSTANCE_NAME`_mstrWrBufSize) {
        `$INSTANCE_NAME`_DATA_REG = `$INSTANCE_NAME`_mstrWrBufPtr[
                                            `$INSTANCE_NAME`_mstrWrBufIndex++];
        `$INSTANCE_NAME`_CSR_REG = `$INSTANCE_NAME`_CSR_TRANSMIT;
    } else {
        `$INSTANCE_NAME`_MCSR_REG = `$INSTANCE_NAME`_MCSR_STOP_GEN;
        `$INSTANCE_NAME`_CSR_REG  = `$INSTANCE_NAME`_CSR_TRANSMIT;
    }
}

void `$INSTANCE_NAME`_Setup(void) {
    uint8_t intState;

    `$INSTANCE_NAME`_CFG_REG  = `$INSTANCE_NAME`_CFG_EN_MSTR;
    `$INSTANCE_NAME`_XCFG_REG = `$INSTANCE_NAME`_XCFG_CLK_EN;
    `$INSTANCE_NAME`_CLKDIV1_REG = LO8(`$INSTANCE_NAME`_DIVIDER);
    `$INSTANCE_NAME`_CLKDIV2_REG = HI8(`$INSTANCE_NAME`_DIVIDER);

    /* Enable power to block */
    intState = CyEnterCriticalSection();
    `$INSTANCE_NAME`_ACT_PWRMGR_REG  |= `$INSTANCE_NAME`_ACT_PWR_EN;
    `$INSTANCE_NAME`_STBY_PWRMGR_REG |= `$INSTANCE_NAME`_STBY_PWR_EN;
    CyExitCriticalSection(intState);

    CyIntSetPriority(`$INSTANCE_NAME`_ISR_NUMBER, `$INSTANCE_NAME`_ISR_PRIORITY);
    CyIntSetVector(`$INSTANCE_NAME`_ISR_NUMBER, `$INSTANCE_NAME`_ISR);
    CyIntEnable(`$INSTANCE_NAME`_ISR_NUMBER);
}

void `$INSTANCE_NAME`_Write(uint8 addr, uint8 len, uint8 *wr_data) {
    `$INSTANCE_NAME`_mstrWrBufIndex = 0;
    `$INSTANCE_NAME`_mstrWrBufSize  = len;
    `$INSTANCE_NAME`_mstrWrBufPtr   = wr_data;

    /* Hardware actions: write address and generate Start or ReStart */
    `$INSTANCE_NAME`_DATA_REG = addr << 1;
    `$INSTANCE_NAME`_MCSR_REG = `$INSTANCE_NAME`_MCSR_START_GEN;
}
