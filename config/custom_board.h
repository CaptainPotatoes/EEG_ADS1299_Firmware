
#ifndef CUSTOM_BRD_H
#define CUSTOM_BRD_H

#ifdef BOARD_ECG_MPU_V1_0//BOARD_ECG_MPU_V1_0
#include "ecg_mpu_custom_v1_0.h"
#else
#error "Custom board definitions not found"
#endif

#endif // BOARD_ECG_MPU_V1_0

