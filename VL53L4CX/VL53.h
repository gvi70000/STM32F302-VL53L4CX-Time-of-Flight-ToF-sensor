#ifndef VL53_H
#define VL53_H

#ifdef __cplusplus
extern "C" {
#endif

#include "vl53lx_error_codes.h"
#include "vl53lx_api.h"

#define I2C_TIMEOUT			200
#define TIME_BUDGET_US	199238 //200ms
#define TIME_BUDGET_MS	200 //200ms
#define AVG_COUNTER			25
#define CAL_DISTANCE		600 //600mm

/* BSP Common Error codes */
#define BSP_ERROR_NONE                    0
#define BSP_ERROR_NO_INIT                -1
#define BSP_ERROR_WRONG_PARAM            -2
#define BSP_ERROR_BUSY                   -3
#define BSP_ERROR_PERIPH_FAILURE         -4
#define BSP_ERROR_COMPONENT_FAILURE      -5
#define BSP_ERROR_UNKNOWN_FAILURE        -6
#define BSP_ERROR_UNKNOWN_COMPONENT      -7
#define BSP_ERROR_BUS_FAILURE            -8
#define BSP_ERROR_CLOCK_FAILURE          -9
#define BSP_ERROR_MSP_FAILURE            -10
#define BSP_ERROR_FEATURE_NOT_SUPPORTED      -11

/* BSP BUS error codes */

#define BSP_ERROR_BUS_TRANSACTION_FAILURE    -100
#define BSP_ERROR_BUS_ARBITRATION_LOSS       -101
#define BSP_ERROR_BUS_ACKNOWLEDGE_FAILURE    -102
#define BSP_ERROR_BUS_PROTOCOL_FAILURE       -103

#define BSP_ERROR_BUS_MODE_FAULT             -104
#define BSP_ERROR_BUS_FRAME_ERROR            -105
#define BSP_ERROR_BUS_CRC_ERROR              -106
#define BSP_ERROR_BUS_DMA_FAILURE            -107

extern volatile uint8_t ToF_EventDetected;
extern VL53LX_CalibrationData_t calibrationData;

void VL53_Reset();
VL53LX_Error VL53_Init(const uint8_t getDataFirst);
VL53LX_Error VL53_Calibrate();
VL53LX_Error VL53_setCalibration();
VL53LX_Error VL53_getCalibration();
uint16_t VL53_getDistance();

#ifdef __cplusplus
}
#endif

#endif /* VL53_H */