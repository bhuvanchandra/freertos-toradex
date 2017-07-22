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

#include "adc_vf6xx.h"
#include "ccm_vf6xx.h"

void vf610_adc_calculate_rates(adc_init_config_t *info)
{
	unsigned long adck_rate, ipg_rate = ccmIpgBusClk;
	uint32_t vf610_hw_avgs[] = { 1, 4, 8, 16, 32 };
	uint8_t count = 0;

	/*
	 * Calculate ADC sample frequencies
	 * Sample time unit is ADCK cycles. ADCK clk source is ipg clock,
	 * which is the same as bus clock.
	 *
	 * ADC conversion time = SFCAdder + AverageNum x (BCT + LSTAdder)
	 * SFCAdder: fixed to 6 ADCK cycles
	 * AverageNum: 1, 4, 8, 16, 32 samples for hardware average.
	 * BCT (Base Conversion Time): fixed to 25 ADCK cycles for 12 bit mode
	 * LSTAdder(Long Sample Time): fixed to 3 ADCK cycles
	 */
	adck_rate = ipg_rate / info->clkDiv;
	for (count = 0; count < 5; count++)
		info->sample_freq_avail[count] =
			adck_rate / (6 + vf610_hw_avgs[count] * (25 + 3));
}

void ADC_HwInit(ADC_Type* base, adc_init_config_t* initConfig)
{
	assert(initConfig);

	uint32_t cfgData = 0;

	switch (initConfig->clkSel) {
	case VF610_ADCIOC_ALTCLK_SET:
		cfgData |= ADC_CFG_ADICLK(0);
		break;
	case VF610_ADCIOC_ADACK_SET:
		cfgData |= ADC_CFG_ADICLK(1);
		break;
	default:
		break;
	}

	/* low power set for calibration */
	cfgData |= ADC_CFG_ADLPC_MASK;

	/* enable high speed for calibration */
	cfgData |= ADC_CFG_ADHSC_MASK;
/* TODO */
#if 0
	/* voltage reference */
	switch (initConfig->volRef) {
	case VF610_ADCIOC_VR_VREF_SET:
		break;
	case VF610_ADCIOC_VR_VALT_SET:
		cfgData |= ADC_CFG_REFSEL(1);
		break;
	case VF610_ADCIOC_VR_VBG_SET:
		cfgData |= VF610_ADC_REFSEL_VBG;
		break;
	default:
	}
#endif
	/* data overwrite enable */
	if (initConfig->ovwren)
		cfgData |= ADC_CFG_OVWREN_MASK;

	ADC_CFG_REG(base) = cfgData;
	ADC_GC_REG(base) = 0;
}

void ADC_SetSampleRate(ADC_Type* base, adc_init_config_t* initConfig)
{
	uint32_t cfgData, gcData;

	cfgData = ADC_CFG_REG(base);
	gcData = ADC_GC_REG(base);

	/* resolution mode */
	cfgData &= ~ADC_CFG_MODE_MASK;
	switch (initConfig->resMode) {
	case 8:
		cfgData |= ADC_CFG_MODE(0); // 8 bit mode
		break;
	case 10:
		cfgData |= ADC_CFG_MODE(1); // 10 bit mode
		break;
	case 12:
		cfgData |= ADC_CFG_MODE(2); // 12 bit mode
		break;
	default:
		break;
	}

	/* clock select and clock divider  */
	cfgData &= ~(ADC_CFG_ADIV_MASK | ADC_CFG_ADICLK_MASK);
	switch (initConfig->clkDiv) {
	case 1:
		break;
	case 2:
		cfgData |= ADC_CFG_ADIV(1); //VF610_ADC_CLK_DIV2;
		break;
	case 4:
		cfgData |= ADC_CFG_ADIV(2); //VF610_ADC_CLK_DIV4;
		break;
	case 8:
		cfgData |= ADC_CFG_ADIV(3); //VF610_ADC_CLK_DIV8;
		break;
	case 16:
		switch (initConfig->clkSel) {
		case VF610_ADCIOC_BUSCLK_SET:
			cfgData |= ADC_CFG_ADICLK(1) | ADC_CFG_ADIV(3); //VF610_ADC_CLK_DIV8;
			break;
		default:
			break;
		}
		break;
	}

	/* Use the short sample mode */
	cfgData &= ~(ADC_CFG_ADLSMP_MASK | ADC_CFG_ADSTS_MASK);

	/* update hardware average selection */
	cfgData &= ~ADC_CFG_AVGS_MASK;
	gcData &= ~ADC_GC_AVGE_MASK;

	switch (initConfig->sampleRate) {
	case VF610_ADC_SAMPLE_1:
		break;
	case VF610_ADC_SAMPLE_4:
		gcData |= ADC_CFG_AVGS(0); // 4 samples
		break;
	case VF610_ADC_SAMPLE_8:
		gcData |= ADC_GC_AVGE_MASK;
		cfgData |= ADC_CFG_AVGS(1); // 8 samples
		break;
	case VF610_ADC_SAMPLE_16:
		gcData |= ADC_GC_AVGE_MASK;
		cfgData |= ADC_CFG_AVGS(2); // 16 samples
		break;
	case VF610_ADC_SAMPLE_32:
		gcData |= ADC_GC_AVGE_MASK;
		cfgData |= ADC_CFG_AVGS(3); // 32 samples
		break;
	default:
		break;
	}

	ADC_CFG_REG(base) = cfgData;
	ADC_GC_REG(base) = gcData;
}

void ADC_Calibration(ADC_Type* base, adc_init_config_t* initConfig)
{
	uint32_t hc_cfg;

	if (!initConfig->calibration)
		return;

	/* enable calibration interrupt */
	hc_cfg = ADC_HC_AIEN_MASK | 0x1F;
	ADC_HC_REG(base, 0) = hc_cfg;

	ADC_GC_REG(base) |= ADC_GC_CAL_MASK;

	while (!(ADC_GC_REG(base) & ADC_GC_CAL_MASK)) { }
}

void ADC_CfgSet(ADC_Type* base, adc_init_config_t* initConfig)
{
	uint32_t cfgData;

	cfgData = ADC_CFG_REG(base);

	cfgData &= ~ADC_CFG_ADLPC_MASK;

	if (initConfig->lpm)
		cfgData |= ADC_CFG_ADLPC_MASK;

	cfgData &= ~ADC_CFG_ADHSC_MASK;

	ADC_CFG_REG(base) = cfgData;
}

void ADC_LogicChInit(ADC_Type* base, uint8_t logicCh, bool continousConv)
{
	if (continousConv)
		ADC_GC_REG(ADC0_BASE_PTR) |= ADC_GC_ADCO_MASK;
	else
		ADC_GC_REG(ADC0_BASE_PTR) &= ~ADC_GC_ADCO_MASK;

	ADC_HC_REG(ADC0_BASE_PTR, 0) = ADC_HC_AIEN_MASK | ADC_HC_ADCH(logicCh);
}

void ADC_LogicChDeinit(ADC_Type* base, uint8_t logicCh)
{
	if (ADC_GC_REG(ADC0_BASE_PTR) & ADC_GC_ADCO_MASK)
		ADC_GC_REG(ADC0_BASE_PTR) &= ~ADC_GC_ADCO_MASK;

	ADC_HC_REG(ADC0_BASE_PTR, 0) &= ~ADC_HC_AIEN_MASK;
	ADC_HC_REG(ADC0_BASE_PTR, 0) = ADC_HC_ADCH(0x1F);
}

/* TODO */
void ADC_SelectInputCh(ADC_Type* base, uint8_t logicCh, uint8_t inputCh)
{
	assert(logicCh <= adcLogicChSW);
}
