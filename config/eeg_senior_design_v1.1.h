/* Copyright (c) 2016 Musa Mahmood
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

/**@ECG CUSTOM WITH MPU V1.0*/
#ifndef ECG_MPU_BOARD_V1_0_H
#define ECG_MPU_BOARD_V1_0_H

// Board has no LEDs or buttons
#define LEDS_NUMBER    0
#define BUTTONS_NUMBER 0
#define HWFC           false


//These are the correct pin definitions
/**/
//#define SPIM0_SCK_PIN      	13    
//#define SPIM0_MOSI_PIN      14    
//#define SPIM0_MISO_PIN      12    
//#define SPIM0_SS_PIN        15
#if (defined(MPU60x0) || defined(MPU9150) || defined(MPU9255))
//USE DEFINITIONS: MPU9255 TWI1_USE_EASY_DMA = 0
#define MPU_TWI_SCL_PIN 1
#define MPU_TWI_SDA_PIN 0
#define MPU_INT_PIN 5

#endif /**@(defined(MPU60x0) || defined(MPU9150) || defined(MPU9255))*/
/**@CRYSTAL DEFINITION*/
#ifdef S210
#define NRF_CLOCK_LFCLKSRC      NRF_CLOCK_LFCLKSRC_XTAL_20_PPM
#else
#define NRF_CLOCK_LFCLKSRC  {	.source        	= NRF_CLOCK_LF_SRC_RC,							\
															.rc_ctiv 				= 2, 																\
															.rc_temp_ctiv 	= 1, 																\
															.xtal_accuracy 	= NRF_CLOCK_LF_XTAL_ACCURACY_20_PPM}
//#define NRF_CLOCK_LFCLKSRC  {	.source        	= NRF_CLOCK_LF_SRC_RC,\
															.rc_ctiv 				= 16, 																		\
															.rc_temp_ctiv 	= 8, 																\
															.xtal_accuracy 	= NRF_CLOCK_LF_XTAL_ACCURACY_250_PPM}
//#define NRF_CLOCK_LFCLKSRC {.source = NRF_CLOCK_LF_SRC_XTAL,\
														.xtal_accuracy = NRF_CLOCK_LF_XTAL_ACCURACY_250_PPM}
#endif

#endif // ECG_MPU_BOARD_V1_0_H
