/*
 * This file is part of the libopencm3 project.
 *
 * Copyright (C) 2009 Uwe Hermann <uwe@hermann-uwe.de>
 * Copyright (C) 2011 Stephen Caudle <scaudle@doceme.com>
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

#include <libopencm3/cm3/assert.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/spi.h>
#include <libopencm3/stm32/pwr.h>
#include <libopencm3/stm32/flash.h>

void setup_spi(void);
void setup_peripheral_clocks(void);
void setup_main_clock(void);

void update_string(uint8_t *data, uint16_t len);

static void gpio_setup(void)
{
	/* Enable GPIOD clock. */
	/* Manually: */
	// RCC_AHB1ENR |= RCC_AHB1ENR_IOPDEN;
	/* Using API functions: */
	rcc_periph_clock_enable(RCC_GPIOD);
	rcc_periph_clock_enable(RCC_GPIOA);

	/* Set GPIO12 (in GPIO port D) to 'output push-pull'. */
	/* Manually: */
	// GPIOD_CRH = (GPIO_CNF_OUTPUT_PUSHPULL << (((8 - 8) * 4) + 2));
	// GPIOD_CRH |= (GPIO_MODE_OUTPUT_2_MHZ << ((8 - 8) * 4));
	/* Using API functions: */
	gpio_mode_setup(GPIOD, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO12);
}

void setup_main_clock()
{
	const clock_scale_t myclock = { /* 102MHz for my LED Serial Data Stuff */
		.pllm = 8,
		.plln = 408,
		.pllp = 4,
		.pllq = 2,
		.hpre = RCC_CFGR_HPRE_DIV_NONE,
		.ppre1 = RCC_CFGR_PPRE_DIV_2,
		.ppre2 = RCC_CFGR_PPRE_DIV_4,
		.power_save = 0,
		.flash_config = FLASH_ACR_ICE | FLASH_ACR_DCE |
			FLASH_ACR_LATENCY_3WS,
		.apb1_frequency = 12000000,
		.apb2_frequency = 24000000,
	};
	// Set the System Clock to 102MHz!!
	rcc_clock_setup_hse_3v3(&myclock);
}

void setup_peripheral_clocks()
{
	rcc_peripheral_enable_clock(&RCC_AHB1ENR,
				    /* GPIO A */
				    RCC_AHB1ENR_IOPAEN |
				    /* GPIO D */
				    RCC_AHB1ENR_IOPDEN |
				    /* GPIO E */
				    RCC_AHB1ENR_IOPEEN);

	rcc_peripheral_enable_clock(&RCC_AHB2ENR,
				    /* USB OTG */
				    RCC_AHB2ENR_OTGFSEN);

	rcc_peripheral_enable_clock(&RCC_APB2ENR,
				    /* SPI 1 */
				    RCC_APB2ENR_SPI1EN);
}

void setup_spi()
{
	/* chip select */
	gpio_mode_setup(GPIOE, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO3);
	/* set to special function select so the SPI module can control it  */
	gpio_mode_setup(GPIOE, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO3);
	
	gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE,
			/* serial clock */
			GPIO5 |
			/* master in/slave out */
			GPIO6 |
			/* master out/slave in */
			GPIO7 |
			/* master slaveselect out */
			GPIO4 );
	gpio_set_af(GPIOA, GPIO_AF5, GPIO5 | GPIO6 | GPIO7 | GPIO4);	

	spi_disable_crc(SPI1);
	// Set the divider to 16 so that we get 6.4MHz for our LED communication!!
	spi_init_master(SPI1, SPI_CR1_BAUDRATE_FPCLK_DIV_4,
			/* high or low for the peripheral device */
			SPI_CR1_CPOL_CLK_TO_1_WHEN_IDLE,
			/* CPHA: Clock phase: read on rising edge of clock */
			SPI_CR1_CPHA_CLK_TRANSITION_2,
			/* DFF: Date frame format (8 or 16 bit) */
			SPI_CR1_DFF_16BIT,
			/* Most or Least Sig Bit First */
			SPI_CR1_MSBFIRST);
	// Master Mode Only!
	spi_disable_software_slave_management(SPI1);
	spi_enable_ss_output(SPI1);

	//spi_enable_software_slave_management(SPI1);
	spi_set_nss_high(SPI1);

	//spi_clear_mode_fault(SPI1);

	spi_enable(SPI1);
}

int main(void)
{
	int i, j;
	setup_main_clock();
	gpio_setup();
        setup_peripheral_clocks();

	setup_spi();
	uint8_t test_bytes[3] = {0, 0, 0};

	/* Blink the LED (PC8) on the board. */
	while (1) {
		/* Manually: */
		// GPIOD_BSRR = GPIO12;		/* LED off */
		// for (i = 0; i < 1000000; i++)	/* Wait a bit. */
		//	__asm__("nop");
		// GPIOD_BRR = GPIO12;		/* LED on */
		// for (i = 0; i < 1000000; i++)	/* Wait a bit. */
		//	__asm__("nop");

		/* Using API functions gpio_set()/gpio_clear(): */
		// gpio_set(GPIOD, GPIO12);	/* LED off */
		// for (i = 0; i < 1000000; i++)	/* Wait a bit. */
		//	__asm__("nop");
		// gpio_clear(GPIOD, GPIO12);	/* LED on */
		// for (i = 0; i < 1000000; i++)	/* Wait a bit. */
		//	__asm__("nop");

		/* Using API function gpio_toggle(): */
		gpio_toggle(GPIOD, GPIO12);	/* LED on/off */
		
		//spi_write(SPI1, 0xFF);
		//spi_write(SPI1, 0x00);
		//spi_write(SPI1, 0xAA);
		//spi_write(SPI1, 0x00);
		
		
		update_string(test_bytes, 3);
		test_bytes[0] += 1;

		//for(j = 0; j < 9; j++)
		//{
		//	test_bytes[j] += 1;
		//}
		
		for (i = 0; i < 200000; i++) {	/* Wait a bit. */
			__asm__("nop");
		}
	}

	return 0;

}

/* turn bits into pulses of the correct ratios for the WS2811 by making *
 * bytes with the correct number of ones & zeros in the right order.    */
void update_string(uint8_t *data, uint16_t len)
{
	uint16_t i = 0;
	int16_t j = 0;
	uint8_t tmp = 0;
		
	for(i=0; i < len; i++)
	{
		tmp = data[i];
		for(j = 7; j > 0; j--)
		{
			if (tmp & (0x01 << j))
			{
				// generate the sequence to represent a 'one' to the WS2811.
				spi_send(SPI1, 0xFFF0);
			}
			else
			{
				// generate the sequence to represent a 'zero'.
				spi_send(SPI1, 0xE000);
			}
		}
	}
}
