

#define `$INSTANCE_NAME`_DPLL_CONTROL_REG \
    (*(reg8  *) `$INSTANCE_NAME`_B_SDLC_dpll_dco__CONTROL_AUX_CTL_REG)
#define `$INSTANCE_NAME`_CD_THRESHOLD_REG \
    (*(reg8  *) `$INSTANCE_NAME`_B_SDLC_oversample_dp_u0__D0_REG)
#define `$INSTANCE_NAME`_CD_LIMIT_REG \
    (*(reg8  *) `$INSTANCE_NAME`_B_SDLC_oversample_dp_u0__D1_REG)
#define `$INSTANCE_NAME`_BIT_COUNT_REG \
    (*(reg8  *) `$INSTANCE_NAME`_B_SDLC_sdlc_dp_bit_counter__CONTROL_AUX_CTL_REG)
#define `$INSTANCE_NAME`_ZERO_INS_COUNT_REG \
    (*(reg8  *) `$INSTANCE_NAME`_B_SDLC_sdlc_dp_zero_ins_counter__CONTROL_AUX_CTL_REG)
#define `$INSTANCE_NAME`_CRC_AUX_CTL_REG \
    (*(reg16 *) `$INSTANCE_NAME`_B_SDLC_sdlc_dp_dp_u0__16BIT_DP_AUX_CTL_REG)
#define `$INSTANCE_NAME`_CRC_POLY_REG \
    (*(reg16 *) `$INSTANCE_NAME`_B_SDLC_sdlc_dp_dp_u0__16BIT_D0_REG)
#define `$INSTANCE_NAME`_CRC_MATCH_REG \
    (*(reg16 *) `$INSTANCE_NAME`_B_SDLC_sdlc_dp_dp_u0__16BIT_D1_REG)
#define `$INSTANCE_NAME`_STATE_CRC_REG \
    (*(reg8  *) `$INSTANCE_NAME`_B_SDLC_tx_ctrl_dp_u0__D0_REG)
#define `$INSTANCE_NAME`_STATE_SYNC_REG \
    (*(reg8  *) `$INSTANCE_NAME`_B_SDLC_tx_ctrl_dp_u0__D1_REG)

#define `$INSTANCE_NAME`_TX_LENGTH_REG \
    (*(reg8  *) `$INSTANCE_NAME`_B_SDLC_tx_ctrl_dp_u0__A0_REG)
#define `$INSTANCE_NAME`_TX_PREAMBLE_REG \
    (*(reg8  *) `$INSTANCE_NAME`_B_SDLC_tx_ctrl_dp_u0__F1_REG)

#define `$INSTANCE_NAME`_RX_DATA_REG \
    `$INSTANCE_NAME`_B_SDLC_sdlc_dp_dp_u0__16BIT_F0_REG
#define `$INSTANCE_NAME`_TX_DATA_REG \
    `$INSTANCE_NAME`_B_SDLC_sdlc_dp_dp_u0__16BIT_F1_REG

void `$INSTANCE_NAME`_Setup(void);
void `$INSTANCE_NAME`_SendReceive(uint8 tx_len, uint8 rx_len, uint8 *tx_data, uint8 *rx_data);
uint16 `$INSTANCE_NAME`_GetRxBytes(void);
