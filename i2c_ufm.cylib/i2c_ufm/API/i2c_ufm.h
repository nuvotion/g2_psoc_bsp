
/* XCFG I2C Extended Configuration Register */
#define `$INSTANCE_NAME`_XCFG_CLK_EN        (0x80u)

/* CFG I2C Configuration Register */
#define `$INSTANCE_NAME`_CFG_EN_MSTR        (0x02u)

/* CSR I2C Control and Status Register */
#define `$INSTANCE_NAME`_CSR_TRANSMIT       (0x04u)

/* MCSR I2C Master Control and Status Register */
#define `$INSTANCE_NAME`_MCSR_STOP_GEN      (0x10u)
#define `$INSTANCE_NAME`_MCSR_START_GEN     (0x01u)

#define `$INSTANCE_NAME`_DIVIDER            ((BCLK__BUS_CLK__MHZ+15)/16)

#define `$INSTANCE_NAME`_ISR_NUMBER         ((uint8) `$INSTANCE_NAME`_IRQ__INTC_NUMBER)
#define `$INSTANCE_NAME`_ISR_PRIORITY       ((uint8) `$INSTANCE_NAME`_IRQ__INTC_PRIOR_NUM)

#define `$INSTANCE_NAME`_ACT_PWR_EN         ((uint8) `$INSTANCE_NAME`_I2C_FF__PM_ACT_MSK)
#define `$INSTANCE_NAME`_STBY_PWR_EN        ((uint8) `$INSTANCE_NAME`_I2C_FF__PM_STBY_MSK)

#define `$INSTANCE_NAME`_ACT_PWRMGR_REG     (*(reg8 *) `$INSTANCE_NAME`_I2C_FF__PM_ACT_CFG)
#define `$INSTANCE_NAME`_STBY_PWRMGR_REG    (*(reg8 *) `$INSTANCE_NAME`_I2C_FF__PM_STBY_CFG)
#define `$INSTANCE_NAME`_XCFG_REG           (*(reg8 *) `$INSTANCE_NAME`_I2C_FF__XCFG)
#define `$INSTANCE_NAME`_CFG_REG            (*(reg8 *) `$INSTANCE_NAME`_I2C_FF__CFG)
#define `$INSTANCE_NAME`_CSR_REG            (*(reg8 *) `$INSTANCE_NAME`_I2C_FF__CSR)
#define `$INSTANCE_NAME`_DATA_REG           (*(reg8 *) `$INSTANCE_NAME`_I2C_FF__D)
#define `$INSTANCE_NAME`_MCSR_REG           (*(reg8 *) `$INSTANCE_NAME`_I2C_FF__MCSR)
#define `$INSTANCE_NAME`_CLKDIV1_REG        (*(reg8 *) `$INSTANCE_NAME`_I2C_FF__CLK_DIV1)
#define `$INSTANCE_NAME`_CLKDIV2_REG        (*(reg8 *) `$INSTANCE_NAME`_I2C_FF__CLK_DIV2)

void `$INSTANCE_NAME`_SetupDMA(uint8 max_len);
void `$INSTANCE_NAME`_SetupIRQ(void);
void `$INSTANCE_NAME`_WriteDMA(uint8 len, uint8 *wr_data);
void `$INSTANCE_NAME`_WriteIRQ(uint8 addr, uint8 len, uint8 *wr_data);
