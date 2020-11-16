#define `$INSTANCE_NAME`_ISR_NUMBER     ((uint8) `$INSTANCE_NAME`_IRQ__INTC_NUMBER)
#define `$INSTANCE_NAME`_ISR_PRIORITY   ((uint8) `$INSTANCE_NAME`_IRQ__INTC_PRIOR_NUM)

#define `$INSTANCE_NAME`_CFG_REG        (*(reg8 *) `$INSTANCE_NAME`_bI2C_UDB_SyncCtl_CtrlReg__CONTROL_REG)
#define `$INSTANCE_NAME`_INT_MASK_REG   (*(reg8 *) `$INSTANCE_NAME`_bI2C_UDB_StsReg__MASK_REG)
#define `$INSTANCE_NAME`_INT_ENABLE_REG (*(reg8 *) `$INSTANCE_NAME`_bI2C_UDB_StsReg__STATUS_AUX_CTL_REG)
#define `$INSTANCE_NAME`_MCLK_PRD_REG   (*(reg8 *) `$INSTANCE_NAME`_bI2C_UDB_Master_ClkGen_u0__D0_REG)
#define `$INSTANCE_NAME`_MCLK_CMP_REG   (*(reg8 *) `$INSTANCE_NAME`_bI2C_UDB_Master_ClkGen_u0__D1_REG)
#define `$INSTANCE_NAME`_DATA_REG       (*(reg8 *) `$INSTANCE_NAME`_bI2C_UDB_Shifter_u0__A0_REG)
#define `$INSTANCE_NAME`_GO_REG         (*(reg8 *) `$INSTANCE_NAME`_bI2C_UDB_Shifter_u0__F1_REG)

void `$INSTANCE_NAME`_SetupDMA(uint8 max_len);
void `$INSTANCE_NAME`_SetupIRQ(void);
void `$INSTANCE_NAME`_WriteDMA(uint8 len, uint8 *wr_data);
void `$INSTANCE_NAME`_WriteIRQ(uint8 addr, uint8 len, uint8 *wr_data);
