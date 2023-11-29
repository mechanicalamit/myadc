/*
 * This file is part of the libopencm3 project.
 *
 * Copyright (C) 2010 Thomas Otto <tommi@viadmin.org>
 * Copyright (C) 2012 Piotr Esden-Tempski <piotr@esden.net>
 * Copyright (C) 2012 Stephen Dwyer <dwyer.sc@gmail.com>
 *
 * This library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this library.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "FreeRTOS.h"
#include "task.h"

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/adc.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/cm3/nvic.h>

/* TODO : Setup Data Structures, long queue for ADC, short queue for Filtering, One value for UART */
volatile uint16_t myadc_val = 0;

static void usart_setup(void)
{
	/* Enable clocks for GPIO port A (for GPIO_USART1_TX) and USART1. */
	rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_USART2);

	/* Setup GPIO pin GPIO_USART1_TX/GPIO9 on GPIO port A for transmit. */
	gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ,
		      GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_USART2_TX);

	/* Setup UART parameters. */
	usart_set_baudrate(USART2, 115200);
	usart_set_databits(USART2, 8);
	usart_set_stopbits(USART2, USART_STOPBITS_1);
	usart_set_mode(USART2, USART_MODE_TX_RX);
	usart_set_parity(USART2, USART_PARITY_NONE);
	usart_set_flow_control(USART2, USART_FLOWCONTROL_NONE);

	/* Finally enable the USART. */
	usart_enable(USART2);
}

static void irq_setup(void)
{
	/* Enable the adc1_2_isr() routine */
	nvic_set_priority(NVIC_ADC1_2_IRQ, 0x00); /* Top 4 bits set priority */
	nvic_enable_irq(NVIC_ADC1_2_IRQ);
}

static void adc_setup(void)
{
	int i; 

	/* Enable GPIO clocks. A for ADC, C for LED */
	rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_GPIOC);

	/* Setup the LEDs. */
	gpio_set_mode(GPIOC, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL,
		      GPIO13);

	/* Setup GPIO for ADC Pin PA0 and maybe PA1 */
	gpio_set_mode(GPIOA, GPIO_MODE_INPUT, GPIO_CNF_INPUT_ANALOG, GPIO0);
	gpio_set_mode(GPIOA, GPIO_MODE_INPUT, GPIO_CNF_INPUT_ANALOG, GPIO1);

	rcc_periph_clock_enable(RCC_ADC1);

	adc_power_off(ADC1);

	/* We configure everything for one single timer triggered injected conversion with interrupt generation. */
	/* While not needed for a single channel, try out scan mode which does all channels in one sweep and
	 * generates the interrupt/EOC/JEOC flags set at the end of all channels, not each one.
	 */
	adc_disable_scan_mode(ADC1);
	adc_set_single_conversion_mode(ADC1);

	/* We want to start the injected conversion with the TIM2 TRGO 
	adc_enable_external_trigger_injected(ADC1,ADC_CR2_JEXTSEL_TIM2_TRGO); */

	adc_disable_external_trigger_regular(ADC1);

	/* Generate the ADC1_2_IRQ NOT Injected now */
	/* adc_enable_eoc_interrupt_injected(ADC1); */
	adc_enable_eoc_interrupt(ADC1);
	adc_set_right_aligned(ADC1);

	adc_set_sample_time_on_all_channels(ADC1, ADC_SMPR_SMP_28DOT5CYC);

	adc_power_on(ADC1);

	/* Wait for ADC starting up. */
	for (i = 0; i < 800000; i++) /* Wait a bit. */
		__asm__("nop");

	adc_reset_calibration(ADC1);
	adc_calibrate(ADC1);
}

static void my_usart_print_int(uint32_t usart, int value)
{
	int8_t i;
	uint8_t nr_digits = 0;
	char buffer[25];

	if (value < 0) {
		usart_send_blocking(usart, '-');
		value = value * -1;
	}

	while (value > 0) {
		buffer[nr_digits++] = "0123456789"[value % 10];
		value /= 10;
	}

	for (i = (nr_digits - 1); i >= 0; i--) {
		usart_send_blocking(usart, buffer[i]);
	}

	usart_send_blocking(usart, '\r');
	usart_send_blocking(usart, '\n');
}

static void task_ADC1(void *arg __attribute((unused)))
{
	static uint8_t channel_array[16];
	const TickType_t xDelay = 500 / portTICK_PERIOD_MS;

	/* Select the channel we want to convert, ADC0 == PA0 */
	channel_array[0] = 0;
	adc_set_regular_sequence(ADC1, 1, channel_array);

	for (;;) {
		gpio_toggle(GPIOC, GPIO13); /* Heartbeat */
		adc_start_conversion_direct(ADC1);
		/* adc1_2_isr :  IRQ will handle the read part */
		vTaskDelay(xDelay);
	}
}

int main(void)
{
	rcc_clock_setup_pll(&rcc_hse_configs[RCC_CLOCK_HSE8_72MHZ]);

	/* Setup UART */
	usart_setup();

	/* Enable IRQs */
	irq_setup();

	/* Setup ADC */
	adc_setup();

	/* Start UART Task, Consumers first */

	/* Start filtering Task */

	/* Send a message on USART1. */
	usart_send_blocking(USART2, 's');
	usart_send_blocking(USART2, 't');
	usart_send_blocking(USART2, 'm');
	usart_send_blocking(USART2, '\r');
	usart_send_blocking(USART2, '\n');

	usart_send_blocking(USART2, 'e');

	/* Start ADC Task */
	xTaskCreate(task_ADC1, "startADC_Conv", 300, NULL, 1, NULL);

	vTaskStartScheduler();
	for (;;)
		;

	return 0;
}

void adc1_2_isr(void)
{
	/* TODO : Read and it to queue */
	myadc_val = adc_read_regular(ADC1);
	usart_send_blocking(USART2, 'd');
}

/* vim: tabstop=4 :set noexpandtab: */