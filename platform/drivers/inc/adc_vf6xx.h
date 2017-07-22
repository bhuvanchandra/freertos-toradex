/*
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * Copyright (c) 2016, Toradex AG
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of Freescale Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef __ADC_IMX7D_H__
#define __ADC_IMX7D_H__

#include <stdint.h>
#include <stdbool.h>
#include <assert.h>
#include "device_imx.h"

/*
 * - Calibrate ADC
 * - Update the configuration register ADC_CFG  Reg
 * - Update Genral control register ADC_GC Reg
 * - Update Trigger control register ADC_HCn Reg
 */

enum clk_sel {
	VF610_ADCIOC_BUSCLK_SET,
	VF610_ADCIOC_ALTCLK_SET,
	VF610_ADCIOC_ADACK_SET,
};

enum average_sel {
	VF610_ADC_SAMPLE_1,
	VF610_ADC_SAMPLE_4,
	VF610_ADC_SAMPLE_8,
	VF610_ADC_SAMPLE_16,
	VF610_ADC_SAMPLE_32,
};

enum conversion_mode_sel {
	VF610_ADC_CONV_NORMAL,
	VF610_ADC_CONV_HIGH_SPEED,
	VF610_ADC_CONV_LOW_POWER,
};

enum lst_adder_sel {
	VF610_ADCK_CYCLES_3,
	VF610_ADCK_CYCLES_5,
	VF610_ADCK_CYCLES_7,
	VF610_ADCK_CYCLES_9,
	VF610_ADCK_CYCLES_13,
	VF610_ADCK_CYCLES_17,
	VF610_ADCK_CYCLES_21,
	VF610_ADCK_CYCLES_25,
};

/*!
 * @addtogroup adc_imx7d_driver
 * @{
 */

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*!
 *  @brief ADC module initialize structure.
 */
typedef struct _adc_init_config
{
	uint32_t	sampleRate;
	uint32_t	clkSel;
	uint32_t	clkDiv;
	uint32_t	volRef;
	bool		calibration;
	bool		ovwren;
	bool		lpm;
	uint8_t		resMode;
	uint32_t	sample_freq_avail[5];
} adc_init_config_t;

/*!
 * @brief ADC logic channel initialize structure.
 */
typedef struct _adc_logic_ch_init_config
{
	uint8_t  inputChannel;          /*!< The logic channel to be set.*/
	bool     coutinuousEnable;      /*!< Continuous sample mode enable configuration.*/
	uint32_t convertRate;           /*!< The continuous rate when continuous sample enabled.*/
	bool     averageEnable;         /*!< Hardware average enable configuration.*/
	uint8_t  averageNumber;         /*!< The average number for hardware average function.*/
} adc_logic_ch_init_config_t;

/*!
 * @brief ADC logic channel selection enumeration.
 */
enum _adc_logic_ch_selection
{
	adcLogicChA     = 0x0,          /*!< ADC Logic Channel A.*/
	adcLogicChB     = 0x1,          /*!< ADC Logic Channel B.*/
	adcLogicChC     = 0x2,          /*!< ADC Logic Channel C.*/
	adcLogicChD     = 0x3,          /*!< ADC Logic Channel D.*/
	adcLogicChSW    = 0x4           /*!< ADC Logic Channel Software.*/
};

/*!
 * @brief ADC hardware average number enumeration.
 */
enum _adc_average_number
{
	adcAvgNum4      = 0x0,          /*!< ADC Hardware Average Number is set to 4.*/
	adcAvgNum8      = 0x1,          /*!< ADC Hardware Average Number is set to 8.*/
	adcAvgNum16     = 0x2,          /*!< ADC Hardware Average Number is set to 16.*/
	adcAvgNum32     = 0x3           /*!< ADC Hardware Average Number is set to 32.*/
};

/*******************************************************************************
 * API
 ******************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif

void vf610_adc_calculate_rates(adc_init_config_t *info);

/*!
 * @name ADC Module Initialization and Configuration functions.
 * @{
 */

/*!
 * @brief Initialize ADC to reset state and initialize with initialize structure.
 *
 * @param base ADC base pointer.
 * @param initConfig ADC initialize structure.
 */
void ADC_HwInit(ADC_Type* base, adc_init_config_t* initConfig);

/*!
 * @brief This function is used to set ADC module sample rate.
 *
 * @param base ADC base pointer.
 * @param sampleRate Desired ADC sample rate.
 */
void ADC_SetSampleRate(ADC_Type* base, adc_init_config_t* initConfig);


void ADC_Calibration(ADC_Type* base, adc_init_config_t* initConfig);

void ADC_CfgSet(ADC_Type* base, adc_init_config_t* initConfig);

/*@}*/

/*!
 * @name ADC Low power control functions.
 * @{
 */

/*!
 * @brief This function is used to stop all digital part power.
 *
 * @param base ADC base pointer.
 * @param clockDown - true:  Clock down.
 *                  - false: Clock running.
 */
void ADC_SetClockDownCmd(ADC_Type* base, bool clockDown);

/*!
 * @brief This function is used to power down ADC analogue core.
 *        Before entering into stop-mode, power down ADC analogue core first.
 * @param base ADC base pointer.
 * @param powerDown - true:  Power down the ADC analogue core.
 *                  - false: Do not power down the ADC analogue core.
 */
void ADC_SetPowerDownCmd(ADC_Type* base, bool powerDown);

/*@}*/

/*!
 * @name ADC Convert Channel Initialization and Configuration functions.
 * @{
 */

/*!
 * @brief Initialize ADC Logic channel with initialize structure.
 *
 * @param base ADC base pointer.
 * @param logicCh ADC module logic channel selection(refer to _adc_logic_ch_selection enumeration).
 * @param chInitConfig ADC logic channel initialize structure.
 */
void ADC_LogicChInit(ADC_Type* base, uint8_t logicCh, bool continousConv);

/*!
 * @brief Reset target ADC logic channel registers to default value.
 *
 * @param base ADC base pointer.
 * @param logicCh ADC module logic channel selection(refer to _adc_logic_ch_selection enumeration).
 */
void ADC_LogicChDeinit(ADC_Type* base, uint8_t logicCh);

/*!
 * @brief Select input channel for target logic channel.
 *
 * @param base ADC base pointer.
 * @param logicCh ADC module logic channel selection(refer to _adc_logic_ch_selection enumeration).
 * @param inputCh Input channel selection for target logic channel(vary from 0 to 15).
 */
void ADC_SelectInputCh(ADC_Type* base, uint8_t logicCh, uint8_t inputCh);

/*!
 * @brief Set ADC conversion rate of target logic channel.
 *
 * @param base ADC base pointer.
 * @param logicCh ADC module logic channel selection(refer to _adc_logic_ch_selection enumeration).
 * @param convertRate ADC conversion rate in Hz.
 */
void ADC_SetConvertRate(ADC_Type* base, uint8_t logicCh, uint32_t convertRate);

/*!
 * @brief Set work state of hardware average feature of target logic channel.
 *
 * @param base ADC base pointer.
 * @param logicCh ADC module logic channel selection(refer to _adc_logic_ch_selection enumeration).
 * @param enable - true: Enable hardware average.
 *               - faluse: Disable hardware average.
 */
void ADC_SetAverageCmd(ADC_Type* base, uint8_t logicCh, bool enable);

/*!
 * @brief Set hardware average number of target logic channel.
 *
 * @param base ADC base pointer.
 * @param logicCh ADC module logic channel selection(refer to _adc_logic_ch_selection enumeration).
 * @param avgNum hardware average number(should select from _adc_average_number enumeration).
 */
void ADC_SetAverageNum(ADC_Type* base, uint8_t logicCh, uint8_t avgNum);

/*@}*/

/*!
 * @name ADC Conversion Control functions.
 * @{
 */

/*!
 * @brief Set continuous convert work mode of target logic channel.
 *
 * @param base ADC base pointer.
 * @param logicCh ADC module logic channel selection(refer to _adc_logic_ch_selection enumeration).
 * @param enable - true:  Enable continuous convert.
 *               - false: Disable continuous convert.
 */
void ADC_SetConvertCmd(ADC_Type* base, uint8_t logicCh, bool enable);

/*!
 * @brief Trigger single time convert on target logic channel.
 *
 * @param base ADC base pointer.
 * @param logicCh ADC module logic channel selection(refer to _adc_logic_ch_selection enumeration).
 */
void ADC_TriggerSingleConvert(ADC_Type* base, uint8_t logicCh);

/*!
 * @brief Get 12-bit length right aligned convert result.
 *
 * @param base ADC base pointer.
 * @param logicCh ADC module logic channel selection(refer to _adc_logic_ch_selection enumeration).
 * @return convert result on target logic channel.
 */
 static inline  uint16_t ADC_GetResult(ADC_Type* base)
 {
 	return (ADC_R_REG(base, 0) & ADC_R_D_MASK);
 }
/*@}*/

#ifdef __cplusplus
}
#endif

/*! @}*/

#endif /* __ADC_IMX7D_H__ */
/*******************************************************************************
 * EOF
 ******************************************************************************/
