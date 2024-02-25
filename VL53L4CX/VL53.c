#include "VL53.h"
#include "i2c.h"
#include "gpio.h"

#include "vl53lx_api.h"
#include "vl53lx_error_codes.h"
#include "vl53l4cx.h"
const uint8_t VL53_ADDRESS = 0x52;

volatile uint8_t ToF_EventDetected = 0;

VL53L4CX_Object_t myVL53 = {0};
VL53L4CX_RANGING_SENSOR_Drv_t myVL53_Drv = {0};
VL53LX_CalibrationData_t calibrationData = {0};

static int I2C1_Init(void) {
	// The init is don in I2C
  return 0;
}

static int I2C1_DeInit(void) {
	// The deinit is don in I2C
  return 0;
}

static int32_t getTick(void) {
  return HAL_GetTick();
}

static int I2C1_Send(const uint16_t DevAddr, uint8_t *pData, const uint16_t Length) {
  VL53LX_Error ret = VL53LX_ERROR_NONE;
  if(HAL_I2C_Master_Transmit(&hi2c1, DevAddr, pData, Length, I2C_TIMEOUT) != HAL_OK)  {
    if(HAL_I2C_GetError(&hi2c1) != HAL_I2C_ERROR_AF) {
      ret = BSP_ERROR_BUS_ACKNOWLEDGE_FAILURE;
    } else {
      ret =  BSP_ERROR_PERIPH_FAILURE;
    }
  }
  return ret;
}

static int I2C1_Recv(const uint16_t DevAddr, uint8_t *pData, const uint16_t Length) {
	int ret = BSP_ERROR_NONE;
  if(HAL_I2C_Master_Receive(&hi2c1, DevAddr, pData, Length, I2C_TIMEOUT) != HAL_OK) {
    if(HAL_I2C_GetError(&hi2c1) != HAL_I2C_ERROR_AF) {
      ret = BSP_ERROR_BUS_ACKNOWLEDGE_FAILURE;
    } else {
      ret =  BSP_ERROR_PERIPH_FAILURE;
    }
  }
  return ret;
}

static int32_t VL53L4CX_InitData(VL53L4CX_Object_t *pObj) {
	// First RESET the senssor
	VL53_Reset();
	// Set up 
	pObj->IO.Init      = I2C1_Init;
	pObj->IO.DeInit    = I2C1_DeInit;
	pObj->IO.Address   = VL53_ADDRESS;
	pObj->IO.WriteReg  = I2C1_Send;
	pObj->IO.ReadReg   = I2C1_Recv;
	pObj->IO.GetTick   = getTick;
	// Wait for the device to boot
	if(VL53LX_WaitDeviceBooted(pObj) != VL53LX_ERROR_NONE) {
		return VL53L4CX_ERROR;
	}
	// Initialize device data
	if(VL53LX_DataInit(pObj) != VL53LX_ERROR_NONE) {
		return VL53L4CX_ERROR;
	}
	// Initialization successful, set object state
	pObj->IsRanging = 0;
	pObj->IsBlocking = 0;
	pObj->IsContinuous = 0;
	pObj->IsAmbientEnabled = 0;
	pObj->IsSignalEnabled = 0;
	pObj->IsInitialized = 1;

	return VL53L4CX_OK;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
  if(GPIO_Pin == INT_Pin) {
    ToF_EventDetected = 1;
  }
}

void VL53_Reset() {
  HAL_GPIO_WritePin(XSHUT_GPIO_Port, XSHUT_Pin, GPIO_PIN_RESET);
  HAL_Delay(5);
  HAL_GPIO_WritePin(XSHUT_GPIO_Port, XSHUT_Pin, GPIO_PIN_SET);
  HAL_Delay(5);
}

VL53LX_Error VL53_Init(const uint8_t getDataFirst) {
	uint32_t id;
	VL53L4CX_Capabilities_t pCap;
	VL53LX_Error ret = VL53L4CX_InitData(&myVL53);
	//Start by confirming device ID
	if(ret == VL53L4CX_OK) ret = VL53L4CX_ReadID(&myVL53, &id);
	if(ret == VL53L4CX_OK && id == VL53L4CX_ID) ret = VL53L4CX_GetCapabilities(&myVL53, &pCap);
	if(getDataFirst) {
		if(ret == VL53L4CX_OK) ret = VL53LX_GetCalibrationData(&myVL53, &calibrationData);
	}
	if(ret == VL53L4CX_OK) ret =  VL53LX_SetCalibrationData(&myVL53, &calibrationData);
	if(ret == VL53L4CX_OK) ret = VL53LX_SetDistanceMode(&myVL53, VL53LX_DISTANCEMODE_LONG);
	if(ret == VL53L4CX_OK) ret = VL53LX_StopMeasurement(&myVL53);
	// SetUp for higher accuracy
	if(ret == VL53L4CX_OK) ret = VL53LX_set_tuning_parm(&myVL53, VL53LX_TUNINGPARM_PHASECAL_PATCH_POWER, 2);
	if(ret == VL53L4CX_OK) ret = VL53LX_set_tuning_parm(&myVL53, VL53LX_TUNINGPARM_RESET_MERGE_THRESHOLD, 16000);
	if(ret == VL53L4CX_OK) ret = VL53LX_SmudgeCorrectionEnable(&myVL53, VL53LX_SMUDGE_CORRECTION_CONTINUOUS);
	if(ret == VL53L4CX_OK) ret = VL53LX_SetXTalkCompensationEnable(&myVL53, 1);
	if(ret == VL53L4CX_OK) ret = VL53LX_SetOffsetCorrectionMode(&myVL53, VL53LX_OFFSETCORRECTIONMODE_PERVCSEL);
	if(ret == VL53L4CX_OK) ret = VL53LX_SetMeasurementTimingBudgetMicroSeconds(&myVL53, TIME_BUDGET_US);
	// Start to measure
	if(ret == VL53L4CX_OK) ret = VL53L4CX_Start(&myVL53, VL53L4CX_MODE_ASYNC_CONTINUOUS);
	return ret;
}

VL53LX_Error VL53_Calibrate() {
	VL53LX_Error status = VL53LX_ERROR_NONE;
	// 388 bytes in calibrationData, initialized to 0, they will be sent to ESP
	
	// 1. WaitDeviceBooted()
	// 2. DataInit()
	status = VL53L4CX_InitData(&myVL53);
	if(status != VL53LX_ERROR_NONE) {
		return status; // Return immediately if error occurred
	}
	// 3. Perform Reference SPAD (Single Photon Avalanche Diode) Management
	status = VL53LX_PerformRefSpadManagement(&myVL53);
	if(status != VL53LX_ERROR_NONE) {
		return status; // Return immediately if error occurred
	}

	//* 4. Perform Crosstalk Calibration only if SPAD management is successful
	status = VL53LX_PerformXTalkCalibration(&myVL53);
	if(status != VL53LX_ERROR_NONE) {
		return status; // Return immediately if error occurred
	}

	// 5. Perform Offset Calibration only if Crosstalk Calibration is successful
	status = VL53LX_PerformOffsetPerVcselCalibration(&myVL53, CAL_DISTANCE);
	if(status != VL53LX_ERROR_NONE) {
		return status; // Return immediately if error occurred
	}

	// 6. Get Calibration Data only if Offset Calibration is successful
	status = VL53LX_GetCalibrationData(&myVL53, &calibrationData);
	if(status != VL53LX_ERROR_NONE) {
		return status; // Return immediately if error occurred
	}

	// 7. Save the calibration data
	status = VL53LX_SetCalibrationData(&myVL53, &calibrationData);
	// Return the final status, whether success or the first error encountered
	return status;
}

// Called when we get the calibration data from ESP
VL53LX_Error VL53_setCalibration() {
	// Return the final status, whether success or the first error encountered
	return VL53LX_SetCalibrationData(&myVL53, &calibrationData);
}

// Called when we do not get the calibration data from ESP
VL53LX_Error VL53_getCalibration() {
	// Return the final status, whether success or the first error encountered
	return VL53LX_GetCalibrationData(&myVL53, &calibrationData);
}

// Get the measured distance, acll it only when ToF_EventDetected = 1
uint16_t VL53_getDistance() {
	uint16_t distance = 0;
	ToF_EventDetected = 0; // Reset the event detected flag
	VL53LX_Error status = VL53LX_ERROR_NONE;
	static VL53L4CX_Result_t Result; // Static to preserve its value between function calls
	status = VL53L4CX_GetDistance(&myVL53, &Result); // Attempt to get distance measurement
	if(status == VL53LX_ERROR_NONE) { // Proceed only if no error
		//We have only one zone and one target
		distance = Result.ZoneResult[0].Distance[0];
//			for (uint8_t zones = 0; zones < VL53L4CX_MAX_NB_ZONES; zones++) { // Iterate through all zones
//				for (uint8_t targets = 0; targets < Result.ZoneResult[zones].NumberOfTargets; targets++) { // Iterate through all targets within a zone
//					uint16_t distance = Result.ZoneResult[zones].Distance[targets];
//					if(distance > maxDist) { // Check if the current distance is greater than maxDist
//						maxDist = distance; // Update maxDist with the current distance
//					}
//				}
//			}
	}
	return distance; // Return the maximum distance found
}