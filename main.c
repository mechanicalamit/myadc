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
#include "queue.h"

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/adc.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/cm3/nvic.h>

/* Read variable for ADC_regular_read, Even though uint32_t is returned, only the lower 16 bits have the ADC values */
uint16_t myadc_val = 0;

/* TODO : Setup Data Structures, long queue for ADC, short queue for Filtering, One value for UART */

/* Queue to push data from ADC read ISR, unit16_t queue is sufficient */
#define ADC_PUSH_QLEN 50
static QueueHandle_t adc_push_q;

/* Queue to push data from filter task to UART task */
#define FILT_PUSH_QLEN 50
static QueueHandle_t filt_push_q;

/* ADC values Plausibility limits */
#define ADC_VAL_MIN 500
#define ADC_VAL_MAX 50000

/* Delays in Tasks, Consumers(UART & FILT) get higher priority and lower delays
 * Producers(ADC) gets lower priority and higher delays */
#define task_ADC_DELAY 500 /* in milli-seconds */
#define task_FILT_DELAY 400
#define task_UART_DELAY 300

#define task_ADC_PRIO 1
#define task_FILT_PRIO 2
#define task_UART_PRIO 3

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
	nvic_set_priority(NVIC_ADC1_2_IRQ, 0xA0); /* Top 4 bits set priority */
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
	adc_disable_scan_mode(ADC1);
	adc_set_single_conversion_mode(ADC1);
	adc_disable_external_trigger_regular(ADC1);

	/* Generate the ADC1_2_IRQ NOT Injected now */
	adc_enable_eoc_interrupt(ADC1);
	adc_set_right_aligned(ADC1);

	adc_set_sample_time_on_all_channels(ADC1, ADC_SMPR_SMP_239DOT5CYC);

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

static void task_uart(void *arg __attribute((unused)))
{
	static uint16_t locavg; /* Hold a poor mans moving average */
	uint16_t locvar;

	/* Receive from Filt queue, 
	 * Already checked for plausibility, so value is good
	 * Delay and let others run if nothing received */
	for (;;) {
		if (xQueueReceive(filt_push_q, &locvar, 0) == pdTRUE) {
			locavg = ((locavg >> 2) + (locvar >> 2)); /* Cannot overflow also */
		/*	my_usart_print_int(USART2, locavg); */
		}
		else
			vTaskDelay( 1 / portTICK_PERIOD_MS);

		taskYIELD();
		/* vTaskDelay(task_UART_DELAY / portTICK_PERIOD_MS); */
	}
}

static void task_filter(void *arg __attribute((unused)))
{
	uint16_t locvar;

	/* Receive from ADC queue, 
	 * Filter(based on max and min), and send to next queue if received
	 * Delay and let others run if nothing received */
	for (;;) {
		if (xQueueReceive(adc_push_q, &locvar, 0) == pdTRUE) {
			if ((locvar >= ADC_VAL_MIN) && (locvar <= ADC_VAL_MAX)) {
				if (xQueueSend(filt_push_q, &locvar, 0) != pdTRUE)
					/* usart_send_blocking( USART2, 'f');  Failed to send to next(filt) queue */ 
					gpio_toggle(GPIOC, GPIO13); /* Heartbeat 2 */
			} else {
			/* 	usart_send_blocking( USART2, 'l');  We are outside Plausibility limits */ 
				gpio_toggle(GPIOC, GPIO13); /* Heartbeat 2 */
			}
		}
		else
			vTaskDelay( 1 / portTICK_PERIOD_MS);

		taskYIELD();
		/* vTaskDelay(task_FILT_DELAY / portTICK_PERIOD_MS); */
	}
}

static void task_ADC1(void *arg __attribute((unused)))
{
	static uint8_t channel_array[16];

	/* Select the channel we want to convert, ADC0 == PA0 */
	channel_array[0] = 0;
	adc_set_regular_sequence(ADC1, 1, channel_array);

	adc_set_continuous_conversion_mode(ADC1);
	adc_start_conversion_direct(ADC1);
	vTaskDelete(NULL); /* Delete this task */
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

	/* Data Structures, see Declarations at top of file */
	adc_push_q = xQueueCreate(ADC_PUSH_QLEN, sizeof(uint16_t));
	filt_push_q = xQueueCreate(FILT_PUSH_QLEN, sizeof(uint16_t));

	/* Send a message on USART1 to show startup demarcation */
	usart_send_blocking(USART2, 's');
	usart_send_blocking(USART2, 't');
	usart_send_blocking(USART2, 'm');
	usart_send_blocking(USART2, '\r');
	usart_send_blocking(USART2, '\n');
	
	/* Clear LED, so that we know its off. 
	 * Now LED coming ON or toggling means something is not right */
	gpio_clear(GPIOC, GPIO13); 

	/* Consumers get higher task priority so that we don't overflow queues */
	/* Start UART Task, Priority 3 */
	xTaskCreate(task_uart, "task_uart", 300, NULL, task_UART_PRIO, NULL);

	/* Start filtering Task, Priority 2 */
	xTaskCreate(task_filter, "task_filter", 300, NULL, task_FILT_PRIO, NULL);

	/* Start ADC Task, Priority 1 */
	xTaskCreate(task_ADC1, "startADC_Conv", 300, NULL, task_ADC_PRIO, NULL);

	/* All Tasks created */
	/* usart_send_blocking(USART2, 'e'); */

	vTaskStartScheduler();
	for (;;)
		;

	return 0;
}

void adc1_2_isr(void)
{
	/* EOC ISR, read the ADC value and store in a variable
	 * Push the variable onto a queue
	 * If queue is full, drop the value and print('d') for demo purposes */
	myadc_val = adc_read_regular(ADC1);
	if (xQueueSendFromISR(adc_push_q, &myadc_val, NULL) != pdTRUE) {
		gpio_toggle(GPIOC, GPIO13); /* Heartbeat 2 */
		/* usart_send_blocking(USART2, 'd'); */
	}
}

/* vim: tabstop=4 :set noexpandtab: */
