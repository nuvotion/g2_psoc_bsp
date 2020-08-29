#include <project.h>

static volatile uint8  isr_buf_idx;
static volatile uint8 *isr_buf;
static volatile uint8  isr_buf_size;
static uint8 dma_ch;
static uint8 dma_td_first;
static uint8 setup_done;

CY_ISR(`$INSTANCE_NAME`_ISR) {
    if (isr_buf_idx < isr_buf_size) {
        `$INSTANCE_NAME`_DATA_REG = isr_buf[isr_buf_idx++];
        `$INSTANCE_NAME`_CSR_REG  = `$INSTANCE_NAME`_CSR_TRANSMIT;
    } else {
        `$INSTANCE_NAME`_MCSR_REG = `$INSTANCE_NAME`_MCSR_STOP_GEN;
        `$INSTANCE_NAME`_CSR_REG  = `$INSTANCE_NAME`_CSR_TRANSMIT;
    }
}

static void `$INSTANCE_NAME`_Config(void) {
    uint8 intState;

    `$INSTANCE_NAME`_CFG_REG  = `$INSTANCE_NAME`_CFG_EN_MSTR;
    `$INSTANCE_NAME`_XCFG_REG = `$INSTANCE_NAME`_XCFG_CLK_EN;
    `$INSTANCE_NAME`_CLKDIV1_REG = LO8(`$INSTANCE_NAME`_DIVIDER);
    `$INSTANCE_NAME`_CLKDIV2_REG = HI8(`$INSTANCE_NAME`_DIVIDER);

    /* Enable power to block */
    intState = CyEnterCriticalSection();
    `$INSTANCE_NAME`_ACT_PWRMGR_REG  |= `$INSTANCE_NAME`_ACT_PWR_EN;
    `$INSTANCE_NAME`_STBY_PWRMGR_REG |= `$INSTANCE_NAME`_STBY_PWR_EN;
    CyExitCriticalSection(intState);
}

void `$INSTANCE_NAME`_SetupDMA(uint8 max_len) {
    uint8 i, curr_td, next_td;
    static uint8 dma_transmit = `$INSTANCE_NAME`_CSR_TRANSMIT;

    if (setup_done) return;

    `$INSTANCE_NAME`_Config();
    CyIntDisable(`$INSTANCE_NAME`_ISR_NUMBER);

    dma_ch = `$INSTANCE_NAME`_DMA_DmaInitialize(0, 0,
            HI16(CYDEV_SRAM_BASE),
            HI16(CYDEV_PERIPH_BASE));

    curr_td = CyDmaTdAllocate();
    dma_td_first = curr_td;

    for (i = 1; i < max_len*2; i++) {
        if ((i % 2) == 0) {
            CyDmaTdSetAddress(curr_td,
                    LO16((uint32) &dma_transmit),
                    LO16((uint32) &`$INSTANCE_NAME`_CSR_REG));
        }
        next_td = CyDmaTdAllocate();
        CyDmaTdSetConfiguration(curr_td, 1, next_td,
                (i % 2) ? CY_DMA_TD_AUTO_EXEC_NEXT : 0);
        curr_td = next_td;
    }
    CyDmaTdSetConfiguration(curr_td, 1, CY_DMA_DISABLE_TD, 0);
    CyDmaTdSetAddress(curr_td,
            LO16((uint32) &dma_transmit),
            LO16((uint32) &`$INSTANCE_NAME`_CSR_REG));

    setup_done = 1;
}

void `$INSTANCE_NAME`_SetupIRQ(void) {
    if (setup_done) return;

    `$INSTANCE_NAME`_Config();

    CyIntSetPriority(`$INSTANCE_NAME`_ISR_NUMBER, `$INSTANCE_NAME`_ISR_PRIORITY);
    CyIntSetVector(`$INSTANCE_NAME`_ISR_NUMBER, `$INSTANCE_NAME`_ISR);
    CyIntEnable(`$INSTANCE_NAME`_ISR_NUMBER);

    setup_done = 1;
}

void `$INSTANCE_NAME`_WriteDMA(uint8 len, uint8 *wr_data) {
    uint8 i, curr_td, next_td;
    static uint8 dma_stop = `$INSTANCE_NAME`_MCSR_STOP_GEN;

    if (!setup_done) return;

    CyDmaChDisable(dma_ch);

    curr_td = dma_td_first;

    for (i = 2; i <= len*2; i++) {
        if (i == len*2) {
            CyDmaTdSetAddress(curr_td,
                    LO16((uint32) &dma_stop),
                    LO16((uint32) &`$INSTANCE_NAME`_MCSR_REG));
        } else if ((i % 2) == 0) {
            CyDmaTdSetAddress(curr_td,
                    LO16((uint32) &wr_data[i/2]),
                    LO16((uint32) &`$INSTANCE_NAME`_DATA_REG));
        }

        CyDmaTdGetConfiguration(curr_td, NULL, &next_td, NULL); 
        curr_td = next_td;
    }

    CyDmaClearPendingDrq(dma_ch);
    CyDmaChSetInitialTd(dma_ch, dma_td_first);
    CyDmaChEnable(dma_ch, 1);

    `$INSTANCE_NAME`_DATA_REG = wr_data[0];
    `$INSTANCE_NAME`_MCSR_REG = `$INSTANCE_NAME`_MCSR_START_GEN;
}

void `$INSTANCE_NAME`_WriteIRQ(uint8 addr, uint8 len, uint8 *wr_data) {
    if (!setup_done) return;

    isr_buf_idx  = 0;
    isr_buf_size = len;
    isr_buf      = wr_data;

    /* Hardware actions: write address and generate Start or ReStart */
    `$INSTANCE_NAME`_DATA_REG = addr << 1;
    `$INSTANCE_NAME`_MCSR_REG = `$INSTANCE_NAME`_MCSR_START_GEN;
}
