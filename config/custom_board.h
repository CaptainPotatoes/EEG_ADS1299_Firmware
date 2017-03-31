
#ifndef CUSTOM_BRD_H
#define CUSTOM_BRD_H

#ifdef BOARD_EEG_CUSTOM
#include "eeg_senior_design_v1.1.h" //TODO NEED TO DEFINE PINS!
#elif defined(BOARD_NRF_BREAKOUT)
#include "nrf_breakout_v1.h"
#else
#error "Custom board definitions not found"
#endif

#endif // BOARD_ECG_MPU_V1_0

