#include <project.h>

static uint8 tx_ch, tx_td;
static uint8 rx_ch, rx_td;
static uint8 rx_val_ch, rx_val_td;
static uint8 rx_buf[32] __attribute__((aligned(4)));

void `$INSTANCE_NAME`_Setup(void) {
    `$INSTANCE_NAME`_DPLL_CONTROL_REG   = 0x20;
    `$INSTANCE_NAME`_BIT_COUNT_REG      = 0x20;
    `$INSTANCE_NAME`_ZERO_INS_COUNT_REG = 0x20;
    `$INSTANCE_NAME`_CRC_AUX_CTL_REG    = 0x0808;
    `$INSTANCE_NAME`_CRC_POLY_REG       = 0x810;
    `$INSTANCE_NAME`_CRC_MATCH_REG      = 0xE2F0;
    `$INSTANCE_NAME`_STATE_CRC_REG      = 0x10;
    `$INSTANCE_NAME`_STATE_SYNC_REG     = 0x7E;

    tx_ch = `$INSTANCE_NAME`_TX_DMA_DmaInitialize(4, 1,
            HI16(CYDEV_SRAM_BASE),
            HI16(CYDEV_PERIPH_BASE));

    rx_ch = `$INSTANCE_NAME`_RX_DMA_DmaInitialize(2, 1,
            HI16(CYDEV_PERIPH_BASE),
            HI16(CYDEV_SRAM_BASE));

    rx_val_ch = `$INSTANCE_NAME`_RX_VALID_DMA_DmaInitialize(0, 1,
                HI16(CYDEV_SRAM_BASE),
                HI16(CYDEV_SRAM_BASE));

    CyDmaClearPendingDrq(tx_ch);
    CyDmaClearPendingDrq(rx_ch);
    CyDmaClearPendingDrq(rx_val_ch);

    tx_td     = CyDmaTdAllocate();
    rx_td     = CyDmaTdAllocate();
    rx_val_td = CyDmaTdAllocate();

    CyDmaTdSetConfiguration(tx_td, 32, tx_td, TD_INC_SRC_ADR);
    CyDmaChSetInitialTd(tx_ch, tx_td);

    CyDmaTdSetConfiguration(rx_td, 32, CY_DMA_DISABLE_TD, TD_INC_DST_ADR);
    CyDmaTdSetAddress(rx_td,
            LO16((uint32_t) `$INSTANCE_NAME`_RX_DATA_REG),
            LO16((uint32_t) rx_buf));

    CyDmaChEnable(tx_ch, 1u);
    CyDmaChEnable(rx_ch, 1u);
}

void `$INSTANCE_NAME`_SendReceive(uint8 tx_len, uint8 rx_len, uint8 *tx_data, uint8 *rx_data) {
    /* Setup receiver for response */
    CyDmaClearPendingDrq(rx_ch);
    CyDmaChSetInitialTd(rx_ch, rx_td);

    /* Double buffer valid messages using RX checksum */
    CyDmaClearPendingDrq(rx_val_ch);
    CyDmaTdSetConfiguration(rx_val_td, rx_len, CY_DMA_DISABLE_TD,
            TD_INC_SRC_ADR | TD_INC_DST_ADR);
    CyDmaTdSetAddress(rx_val_td,
            LO16((uint32_t) rx_buf),
            LO16((uint32_t) rx_data));
    CyDmaChSetInitialTd(rx_val_ch, rx_val_td);
    CyDmaChEnable(rx_val_ch, 1u);

    /* Send message */
    CyDmaClearPendingDrq(tx_ch);
    CyDmaTdSetAddress(tx_td,
            LO16((uint32_t) tx_data),
            LO16((uint32_t) `$INSTANCE_NAME`_TX_DATA_REG));

    /* Reset FIFOs and reset length */
    `$INSTANCE_NAME`_CRC_AUX_CTL_REG = 0x0B0B;
    `$INSTANCE_NAME`_CRC_AUX_CTL_REG = 0x0808;
    `$INSTANCE_NAME`_TX_LENGTH_REG = (tx_len+2)*8-1;

    CyDmaChSetRequest(tx_ch, CPU_REQ);

    `$INSTANCE_NAME`_TX_PREAMBLE_REG = 0xFF;
    `$INSTANCE_NAME`_TX_PREAMBLE_REG = 0xFF;
    `$INSTANCE_NAME`_TX_PREAMBLE_REG = 0xFF;
    `$INSTANCE_NAME`_TX_PREAMBLE_REG = 0x7E;
}
