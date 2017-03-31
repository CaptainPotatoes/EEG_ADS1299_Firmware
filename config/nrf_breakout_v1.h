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
#ifndef BOARD_NRF_BREAKOUT_H //BOARD_PCA10028
#define BOARD_NRF_BREAKOUT_H

// Board has no LEDs or buttons
#define LEDS_NUMBER    0
#define BUTTONS_NUMBER 0
#define HWFC           false

#if (defined(MPU60x0) || defined(MPU9150) || defined(MPU9255))
//USE DEFINITIONS: MPU9255 TWI1_USE_EASY_DMA = 0

#endif /**@(defined(MPU60x0) || defined(MPU9150) || defined(MPU9255))*/
/**@CRYSTAL DEFINITION*/
#ifdef S210
#define NRF_CLOCK_LFCLKSRC      NRF_CLOCK_LFCLKSRC_XTAL_20_PPM
#else
#define NRF_CLOCK_LFCLKSRC  {	.source        	= NRF_CLOCK_LF_SRC_RC,							\
															.rc_ctiv 				= 2, 																\
															.rc_temp_ctiv 	= 1, 																\
															.xtal_accuracy 	= NRF_CLOCK_LF_XTAL_ACCURACY_20_PPM}

#endif

#endif // BOARD_NRF_BREAKOUT
