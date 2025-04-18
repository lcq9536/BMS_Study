/*
 *  ======== INA237.h ========
 *  INA237 Interface
 */
#ifndef ti_sensors_INA237__include
#define ti_sensors_INA237__include 1

#include <stdint.h>

/* support C++ sources */
#ifdef __cplusplus
extern "C" {
#endif

#define INA237_config_register 0x00U
#define INA237_config_register_rst_NormalOperation 0x0000U
#define INA237_config_register_rst_SystemReset 0x8000U
#define INA237_config_register_reserved0_ENABLE 0x4000U
#define INA237_config_register_reserved0_DISABLE 0x0000U
#define INA237_config_register_reserved1_ENABLE 0x0020U
#define INA237_config_register_reserved1_DISABLE 0x0000U
#define INA237_config_register_adcrange_16384mV 0x0000U
#define INA237_config_register_adcrange_4096mV 0x0010U
#define INA237_adc_config_register 0x01U
#define INA237_adc_config_register_mode_Shutdown0 0x0000U
#define INA237_adc_config_register_mode_Triggeredbusvoltagesingleshot 0x1000U
#define INA237_adc_config_register_mode_Triggeredshuntvoltagesingleshot 0x2000U
#define INA237_adc_config_register_mode_Triggeredshuntvoltageandbusvoltagesingleshot 0x3000U
#define INA237_adc_config_register_mode_Triggeredtemperaturesingleshot 0x4000U
#define INA237_adc_config_register_mode_Triggeredtemperatureandbusvoltagesingleshot 0x5000U
#define INA237_adc_config_register_mode_Triggeredtemperatureandshuntvoltagesingleshot 0x6000U
#define INA237_adc_config_register_mode_Triggeredbusvoltageshuntvoltageandtemperaturesingleshot 0x7000U
#define INA237_adc_config_register_mode_Shutdown1 0x8000U
#define INA237_adc_config_register_mode_Continuousbusvoltageonly 0x9000U
#define INA237_adc_config_register_mode_Continuousshuntvoltageonly 0xA000U
#define INA237_adc_config_register_mode_Continuousshuntandbusvoltage 0xB000U
#define INA237_adc_config_register_mode_Continuoustemperatureonly 0xC000U
#define INA237_adc_config_register_mode_Continuousbusvoltageandtemperature 0xD000U
#define INA237_adc_config_register_mode_Continuoustemperatureandshuntvoltage 0xE000U
#define INA237_adc_config_register_mode_Continuousbusvoltageshuntvoltageandtemperature 0xF000U
#define INA237_adc_config_register_vbusct_50us 0x0000U
#define INA237_adc_config_register_vbusct_84us 0x0200U
#define INA237_adc_config_register_vbusct_150us 0x0400U
#define INA237_adc_config_register_vbusct_280us 0x0600U
#define INA237_adc_config_register_vbusct_540us 0x0800U
#define INA237_adc_config_register_vbusct_1052us 0x0A00U
#define INA237_adc_config_register_vbusct_2074us 0x0C00U
#define INA237_adc_config_register_vbusct_4120us 0x0E00U
#define INA237_adc_config_register_vshct_50us 0x0000U
#define INA237_adc_config_register_vshct_84us 0x0040U
#define INA237_adc_config_register_vshct_150us 0x0080U
#define INA237_adc_config_register_vshct_280us 0x00C0U
#define INA237_adc_config_register_vshct_540us 0x0100U
#define INA237_adc_config_register_vshct_1052us 0x0140U
#define INA237_adc_config_register_vshct_2074us 0x0180U
#define INA237_adc_config_register_vshct_4120us 0x01C0U
#define INA237_adc_config_register_vtct_50us 0x0000U
#define INA237_adc_config_register_vtct_84us 0x0008U
#define INA237_adc_config_register_vtct_150us 0x0010U
#define INA237_adc_config_register_vtct_280us 0x0018U
#define INA237_adc_config_register_vtct_540us 0x0020U
#define INA237_adc_config_register_vtct_1052us 0x0028U
#define INA237_adc_config_register_vtct_2074us 0x0030U
#define INA237_adc_config_register_vtct_4120us 0x0038U
#define INA237_adc_config_register_avg_1 0x0000U
#define INA237_adc_config_register_avg_4 0x0001U
#define INA237_adc_config_register_avg_16 0x0002U
#define INA237_adc_config_register_avg_64 0x0003U
#define INA237_adc_config_register_avg_128 0x0004U
#define INA237_adc_config_register_avg_256 0x0005U
#define INA237_adc_config_register_avg_512 0x0006U
#define INA237_adc_config_register_avg_1024 0x0007U
#define INA237_shunt_cal_register 0x02U
#define INA237_shunt_cal_register_reserved0_ENABLE 0x8000U
#define INA237_shunt_cal_register_reserved0_DISABLE 0x0000U
#define INA237_vshunt_register 0x04U
#define INA237_vbus_register 0x05U
#define INA237_dietemp_register 0x06U
#define INA237_current_register 0x07U
#define INA237_power_register 0x08U
#define INA237_diag_alrt_register 0x0BU
#define INA237_diag_alrt_register_alatch_Transparent 0x0000U
#define INA237_diag_alrt_register_alatch_LatchedAlertpin 0x8000U
#define INA237_diag_alrt_register_cnvr_DisableconversionreadyflagonALERTpin 0x0000U
#define INA237_diag_alrt_register_cnvr_EnablesconversionreadyflagonALERTpin 0x4000U
#define INA237_diag_alrt_register_slowalert_ALERTcomparisononnonaveragedADCvalue 0x0000U
#define INA237_diag_alrt_register_slowalert_ALERTcomparisononaveragedvalue 0x2000U
#define INA237_diag_alrt_register_apol_Normalactivelowopendrain 0x0000U
#define INA237_diag_alrt_register_apol_Invertedactivehighopendrain 0x1000U
#define INA237_diag_alrt_register_mathof_ENABLE 0x0200U
#define INA237_diag_alrt_register_mathof_DISABLE 0x0000U
#define INA237_diag_alrt_register_reserved1_ENABLE 0x0100U
#define INA237_diag_alrt_register_reserved1_DISABLE 0x0000U
#define INA237_diag_alrt_register_tmpol_ENABLE 0x0080U
#define INA237_diag_alrt_register_tmpol_DISABLE 0x0000U
#define INA237_diag_alrt_register_shntol_ENABLE 0x0040U
#define INA237_diag_alrt_register_shntol_DISABLE 0x0000U
#define INA237_diag_alrt_register_shntul_ENABLE 0x0020U
#define INA237_diag_alrt_register_shntul_DISABLE 0x0000U
#define INA237_diag_alrt_register_busol_ENABLE 0x0010U
#define INA237_diag_alrt_register_busol_DISABLE 0x0000U
#define INA237_diag_alrt_register_busul_ENABLE 0x0008U
#define INA237_diag_alrt_register_busul_DISABLE 0x0000U
#define INA237_diag_alrt_register_pol_ENABLE 0x0004U
#define INA237_diag_alrt_register_pol_DISABLE 0x0000U
#define INA237_diag_alrt_register_cnvrf_ENABLE 0x0002U
#define INA237_diag_alrt_register_cnvrf_DISABLE 0x0000U
#define INA237_diag_alrt_register_memstat_ENABLE 0x0001U
#define INA237_diag_alrt_register_memstat_DISABLE 0x0000U
#define INA237_sovl_register 0x0CU
#define INA237_suvl_register 0x0DU
#define INA237_bovl_register 0x0EU
#define INA237_bovl_register_reserved0_ENABLE 0x8000U
#define INA237_bovl_register_reserved0_DISABLE 0x0000U
#define INA237_buvl_register 0x0FU
#define INA237_buvl_register_reserved0_ENABLE 0x8000U
#define INA237_buvl_register_reserved0_DISABLE 0x0000U
#define INA237_temp_limit_register 0x10U
#define INA237_pwr_limit_register 0x11U
#define INA237_manufacturer_id_register 0x3EU


/*
 *  ======== INA237_State ========
 *  Initial configuration state for a INA237 sensor
 */
typedef struct INA237_State {
    uint16_t config; //config_register
    uint16_t adcconfig; //adc_config_register
    uint16_t shuntcal; //shunt_cal_register
    uint16_t diagalrt; //diag_alrt_register
    uint16_t sovl; //sovl_register
    uint16_t suvl; //suvl_registerx
    uint16_t bovl; //bovl_register
    uint16_t buvl; //buvl_register
    uint16_t templimit; //temp_limit_register
    uint16_t pwrlimit; //pwr_limit_register

    uint16_t adcrange; //config_register_adcrange
    float currentlsb; //current lsb value

    uint8_t busId;   /* I2C bus id */
    uint8_t devAddr; /* Sensor's I2C address on the bus */

    uint16_t osWait; /* One shot conversion time (in ms)  */
} INA237_State;

/*
 *  ======== INA237_Handle ========
 *  First argument to all INA237 methods
 */
typedef INA237_State *INA237_Handle;

extern void INA237_Init(int n);
/*
 *  ======== INA237_writeReg ========
  * Write register
  */
extern void INA237_writeReg(INA237_Handle sensor, uint8_t regAddr, uint16_t value);

/*
 *  ======== INA237_config ========
 *  Configure device with current settings
 */
extern void INA237_config(INA237_Handle sensor);

/*
 *  ======== INA237_setCURRENT_LSB ========
 *  Set the CURRENT_LSB value used for calculations
 */
extern void INA237_setCURRENT_LSB(INA237_Handle sensor, float CURRENT_LSB);

/*
 *  ======== INA237_readReg ========
 *  Read register
 */
extern uint64_t INA237_readReg(INA237_Handle sensor, uint8_t regAddr);

/*
 *  ======== INA237_getVSHUNT_mV ========
 *  Get VSHUNT value (mV)
 */
extern float INA237_getVSHUNT_mV(INA237_Handle sensor);

/*
 *  ======== INA237_getVBUS_V ========
 *  Get VBUS value (V)
 */
extern float INA237_getVBUS_V(INA237_Handle sensor);

/*
 *  ======== INA237_getDIETEMP_C ========
 *  Get DIETMEP value (C)
 */
extern float INA237_getDIETEMP_C(INA237_Handle sensor);

/*
 *  ======== INA237_getDIETEMP_F ========
 *  Get DIETMEP value (F)
 */
extern float INA237_getDIETEMP_F(INA237_Handle sensor);

/*
 *  ======== INA237_getCURRENT_signedLSB ========
 *  Get CURRENT value (signed value in LSBs)
 */
extern float INA237_getCURRENT_signedLSB(INA237_Handle sensor);

/*
 *  ======== INA237_getCURRENT_A ========
 *  Get CURRENT value (A)
 */
extern float INA237_getCURRENT_A(INA237_Handle sensor);

/*
 *  ======== INA237_getPOWER_signedLSB ========
 *  Get POWER value (signed value in LSBs)
 */
extern float INA237_getPOWER_signedLSB(INA237_Handle sensor);

/*
 *  ======== INA237_getPOWER_W ========
 *  Get POWER value (W)
 */
extern float INA237_getPOWER_W(INA237_Handle sensor);

/* support C++ sources */
#ifdef __cplusplus
}
#endif

#endif
