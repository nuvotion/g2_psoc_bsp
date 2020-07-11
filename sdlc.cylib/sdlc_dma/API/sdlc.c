#include <project.h>

static uint8 buffer_ch, buffer_td;

void `$INSTANCE_NAME`_Setup(void) {
    `$INSTANCE_NAME`_DPLL_CONTROL_REG   = 0x20;
    `$INSTANCE_NAME`_BIT_COUNT_REG      = 0x20;
    `$INSTANCE_NAME`_ZERO_INS_COUNT_REG = 0x20;
    `$INSTANCE_NAME`_CRC_AUX_CTL_REG    = 0x0808;
    `$INSTANCE_NAME`_CRC_POLY_REG       = 0x810;
    `$INSTANCE_NAME`_CRC_MATCH_REG      = 0xE2F0;
    `$INSTANCE_NAME`_STATE_CRC_REG      = 0x10;
    `$INSTANCE_NAME`_STATE_SYNC_REG     = 0x7E;

    buffer_ch = `$INSTANCE_NAME`_TX_DMA_DmaInitialize(4, 1,
            HI16(CYDEV_SRAM_BASE),
            HI16(CYDEV_PERIPH_BASE));

    CyDmaClearPendingDrq(buffer_ch);

    buffer_td = CyDmaTdAllocate();

    CyDmaTdSetConfiguration(buffer_td, 32, buffer_td, TD_INC_SRC_ADR);

    CyDmaChSetInitialTd(buffer_ch, buffer_td);

    CyDmaChEnable(buffer_ch, 1u);
}

void `$INSTANCE_NAME`_Send(uint8 len, uint8 *data) {
    `$INSTANCE_NAME`_TX_LENGTH_REG = (len+2)*8-1;

    CyDmaClearPendingDrq(buffer_ch);

    CyDmaTdSetAddress(buffer_td,
            LO16((uint32_t) data),
            LO16((uint32_t) `$INSTANCE_NAME`_TX_DATA_REG));

    `$INSTANCE_NAME`_CRC_AUX_CTL_REG = 0x0A0A;
    `$INSTANCE_NAME`_CRC_AUX_CTL_REG = 0x0808;

    CyDmaChSetRequest(buffer_ch, CPU_REQ);

    `$INSTANCE_NAME`_TX_PREAMBLE_REG = 0xFF;
    `$INSTANCE_NAME`_TX_PREAMBLE_REG = 0xFF;
    `$INSTANCE_NAME`_TX_PREAMBLE_REG = 0xFF;
    `$INSTANCE_NAME`_TX_PREAMBLE_REG = 0x7E;
}

