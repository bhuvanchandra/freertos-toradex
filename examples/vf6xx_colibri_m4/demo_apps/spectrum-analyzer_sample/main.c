/*
 * Copyright (c) 2016-17, Toradex AG
 * Bhuvanchandra DV <bhuvanchandra.dv@gmail.com>
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

#include <stdio.h>
#include <errno.h>
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "string.h"
#include "assert.h"
#include "rpmsg/rpmsg.h"
#include "plat_porting.h"
#include "ccm_vf6xx.h"
#include "debug_console_vf6xx.h"
#include "pin_mux.h"
#include "adc_vf6xx.h"
#include "arm_math.h"
#include "arm_const_structs.h"

#include "gpio_vf6xx.h"
#include "pin_mux.h"

#define SAMPLES		1024
#define RFFT_SIZE	SAMPLES / 2

#define VF610_SA_RAW_READ		0
#define VF610_SA_CONT_READ		1

#define READ_MODE(x)	((x) << 4)
#define ADC_CHANNEL(x)  (x & 0xF)

/*
 * APP decided interrupt priority
 */
#define APP_MSCM_IRQ_PRIORITY	3

/*
 * function decalaration for platform provided facility
 */
extern void platform_interrupt_enable(void);
extern void platform_interrupt_disable(void);

/* Internal functions */
static void rpmsg_channel_created(struct rpmsg_channel *rp_chnl);
static void rpmsg_channel_deleted(struct rpmsg_channel *rp_chnl);
static void rpmsg_read_cb(struct rpmsg_channel *, void *, int, void *, unsigned long);

/* Globals */
static struct remote_device *rdev;
static struct rpmsg_channel *app_chnl;

xSemaphoreHandle app_sema, sampling_sema, fft_sema;

uint16_t fftSize = RFFT_SIZE;
uint8_t ifftFlag = 0;
uint8_t doBitReverse = 1;
volatile arm_status status;

q31_t Buffer_In_Real[RFFT_SIZE] = {0};
q31_t Buffer_Out_Complex[SAMPLES] = {0};
q31_t Buffer_Magnitude_Real[RFFT_SIZE] = {0};

arm_rfft_instance_q31 Instance_Real;

/* TODO: Fix the MSCM-IR interrupt routing to M4 too */
void ADC0_Handler(void)
{
	printf("IRQ: ADC Raw val: %d\r\n", ADC_GetResult(ADC0_BASE_PTR));
}

void samplingTask(void *pvParameters)
{
	uint8_t count = 0;

	vSemaphoreCreateBinary(sampling_sema);
	xSemaphoreTake(sampling_sema, portMAX_DELAY);

	while(1) {
		if (ADC_HS_REG(ADC0_BASE_PTR) & ADC_HS_COCO0_MASK) {
			//GPIO_TogglePinOutput(GPIO(39), 39, gpioPinSet);
			Buffer_In_Real[count++] = (q31_t)ADC_GetResult(ADC0_BASE_PTR);
			if (count == (SAMPLES - 1)) {
				count = 0;
				xSemaphoreGive(sampling_sema);
			}
			//GPIO_TogglePinOutput(GPIO(39), 39, gpioPinSet);
		}
		vTaskDelay(1);
	}
}

void fftTransformTask(void *pvParameters)
{
	uint8_t count = 0;
	q31_t maxValue;
	uint32_t testIndex;
	arm_rfft_instance_q31 Instance_Real;

	vSemaphoreCreateBinary(fft_sema);
	xSemaphoreTake(fft_sema, portMAX_DELAY);

	while(1) {
		xSemaphoreTake(sampling_sema, portMAX_DELAY);

		status = arm_rfft_init_q31(&Instance_Real, fftSize, ifftFlag, doBitReverse);
		arm_rfft_q31(&Instance_Real, Buffer_In_Real, Buffer_Out_Complex);

		// Calculate magnitude of FFT complex output

		/* FIXME: Not working */
		//arm_cmplx_mag_q31(Buffer_Out_Complex, Buffer_Magnitude_Real, RFFT_SIZE);

		/* TODO: Use standard function instead */
		for (count = 0; count < SAMPLES; count += 2) {
			Buffer_Magnitude_Real[count / 2] =
				(q31_t)(sqrtf((Buffer_Out_Complex[count] *
				Buffer_Out_Complex[count]) +
				(Buffer_Out_Complex[count + 1] *
				Buffer_Out_Complex[count + 1])));
			//printf("%d\r\n", (int16_t)Buffer_Magnitude_Real[i / 2]);
		}

		/* Calculates maxValue and returns corresponding BIN value */
		arm_max_q31(Buffer_Magnitude_Real, RFFT_SIZE, &maxValue, &testIndex);
		printf("%ld, %d\r\n", testIndex, (uint16_t)maxValue);

		xSemaphoreGive(fft_sema);
		//GPIO_TogglePinOutput(GPIO(38), 38, gpioPinSet);
	}
}

void rpmsgTask(void *pvParameters)
{
	int ret = 0;

	app_sema = xSemaphoreCreateCounting(3, 0); /* TODO: Do we need multiple semaphores ? */

	/*
	* RPMSG Init as REMOTE
	*/
	if (rpmsg_init(0, &rdev, rpmsg_channel_created, rpmsg_channel_deleted, rpmsg_read_cb, RPMSG_MASTER))
		printf("rpmsg_init Failed!\r\n");

	/*
	 * rpmsg_channel_created will post the first semaphore
	 */
	xSemaphoreTake(app_sema, portMAX_DELAY);

	printf("RPMSG initialized as remote\r\n");
	printf("Name service handshake is done b/w A5 and M4\r\nM4 has setup a rpmsg channel [%lu ---> %lu]\r\n", app_chnl->src, app_chnl->dst);

	while (1) {
		xSemaphoreTake(fft_sema, portMAX_DELAY);

		/* FIXME: We are limited with 496B as payload */
		/*
		* Send the data multiple times untill we send all data
		*/
		//GPIO_TogglePinOutput(GPIO(92), 92, gpioPinSet);
		/* Validate device state */
		ret = rpmsg_send(app_chnl, Buffer_Magnitude_Real, (RFFT_SIZE / 2));
		if (ret)
			printf("Send failed with:%d\r\n", ret);

		//if (rpmsg_send(app_chnl, (void*)((uint16_t*)(Buffer_Magnitude_Real + (RFFT_SIZE / 2))), RFFT_SIZE / 2))
		//	printf("Send failed!\r\n");

		//GPIO_TogglePinOutput(GPIO(92), 92, gpioPinSet);

		/*
		* Once a message is consumed, the MSCM receive interrupt can be enabled
		* again
		*/
		platform_interrupt_enable();
	}
}

int main(void)
{
	/* Init Clock Control, UART and ADC*/
	CCM_GetClocks();
	CCM_ControlGate(ccmCcgrGateUart2, ccmClockNeededAll);
	CCM_ControlGate(ccmCcgrGateAdc0, ccmClockNeededRunWait);

	configure_uart_pins(UART2);
	vf6xx_DbgConsole_Init(UART2, ccmIpgBusClk, 115200);
#if DEBUG
	gpio_init_t tp1 = {.pin = 38, .direction = gpioDigitalOutput}; //SODIMM_63
	GPIO_Init(GPIO(38), &tp1);
	gpio_init_t tp2 = {.pin = 39, .direction = gpioDigitalOutput}; // SODIMM_55
	GPIO_Init(GPIO(39), &tp2);
	gpio_init_t tp3 = {.pin = 92, .direction = gpioDigitalOutput}; // SODIMM_100
	GPIO_Init(GPIO(92), &tp3);
#endif
	printf("Spectrum Analyzer Demo\r\n");

	/* Setup ADC init structure. */
	adc_init_config_t adcConfig = {
					.sampleRate = 2,
					.clkSel = 0,
					.clkDiv = 8,
					.volRef = 0,
					.calibration = true,
					.ovwren = true,
					.lpm = true,
					.resMode = 12};

#if DEBUG
	vf610_adc_calculate_rates(&adcConfig);
	printf("Available Sampling Frequencies:\r\n");
	for (int i = 0; i < 5; i++)
		printf("%lu \r\n", adcConfig.sample_freq_avail[i]);
#endif

	ADC_HwInit(ADC0_BASE_PTR, &adcConfig);
	ADC_SetSampleRate(ADC0_BASE_PTR, &adcConfig);
	ADC_Calibration(ADC0_BASE_PTR, &adcConfig);
	ADC_CfgSet(ADC0_BASE_PTR, &adcConfig);

	/* TODO: Fix the MSCM-IR interrupt routing to M4 too */
	//NVIC_EnableIRQ(ADC0_IRQ);
	//NVIC_SetPriority(ADC0_IRQ, 3);

	/*
	* Prepare for the MSCM Interrupt
	* MSCM must be initialized before rpmsg init is called
	*/
	platform_interrupt_enable();
	NVIC_SetPriority(CPU2CPU_INT0_IRQ, APP_MSCM_IRQ_PRIORITY);
	NVIC_SetPriority(CPU2CPU_INT1_IRQ, APP_MSCM_IRQ_PRIORITY);

	xTaskCreate(rpmsgTask, "rpmsg_task", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY+1, NULL);
	xTaskCreate(samplingTask, "ADC_Task", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY+1, NULL);
	xTaskCreate(fftTransformTask, "FFT_Task", configMINIMAL_STACK_SIZE * 2, NULL, tskIDLE_PRIORITY+1, NULL);

	vTaskStartScheduler();

	while (true);
}

/* rpmsg_rx_callback will call into this for a channel creation event */
static void rpmsg_channel_created(struct rpmsg_channel *rp_chnl)
{
	/*
	 * We should give the created rp_chnl handler to app layer
	 */
	app_chnl = rp_chnl;

	/* Validate device state */
	if (app_chnl->state != RPMSG_CHNL_STATE_ACTIVE)
		printf("M4 Channel state Inactive\r\n");
	if (app_chnl->rdev->state != RPMSG_DEV_STATE_ACTIVE)
		printf("A5 Channel state Inactive\r\n");
	/*
	 * Sync to application layer
	 */
	printf("rpmsg channel created\r\n");
	xSemaphoreGiveFromISR(app_sema, NULL);
}

static void rpmsg_channel_deleted(struct rpmsg_channel *rp_chnl)
{
	rpmsg_destroy_ept(rp_chnl->rp_ept);
}

static void rpmsg_read_cb(struct rpmsg_channel *rp_chnl, void *data, int len,
                void * priv, unsigned long src)
{
	uint32_t value, m4_config;
	m4_config = *(uint32_t*)data;

	/*
	* Temperorily Disable MSCM Receive Interrupt to avoid master
	* sending too many messages and remote will fail to keep pace
	* to consume
	*/
	platform_interrupt_disable();

	int channel = (m4_config & 0xF);

	if (m4_config >> 5) {
		if (((m4_config & 0x10) >> 4) == VF610_SA_CONT_READ) {
			ADC_LogicChInit(ADC0_BASE_PTR, channel, VF610_SA_CONT_READ);
			printf("Started sampling on channel:%d\r\n", channel);
		} else if (((m4_config & 0x10) >> 4) == VF610_SA_RAW_READ) {
			ADC_LogicChInit(ADC0_BASE_PTR, channel, VF610_SA_RAW_READ);

			/* FIXME: Ideally this should be in ISR */
			if (ADC_HS_REG(ADC0_BASE_PTR) & ADC_HS_COCO0_MASK) {
				value = ADC_GetResult(ADC0_BASE_PTR);
				if (rpmsg_send(app_chnl, (void*)&value, sizeof(uint32_t)))
					printf("Send failed!\r\n");

				ADC_LogicChDeinit(ADC0_BASE_PTR, channel);
			}
		}
	} else {
		printf("Stopped sampling on channel:%d\r\n", channel);
		ADC_LogicChDeinit(ADC0_BASE_PTR, channel);
	}
}
