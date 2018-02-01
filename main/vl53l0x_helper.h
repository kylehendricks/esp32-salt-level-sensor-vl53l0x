#include "vl53l0x_def.h"
#include "vl53l0x_platform.h"

VL53L0X_Error init_vl53l0x(VL53L0X_Dev_t *pMyDevice);

VL53L0X_Error take_reading(VL53L0X_Dev_t *pMyDevice, VL53L0X_RangingMeasurementData_t *measurementData);
