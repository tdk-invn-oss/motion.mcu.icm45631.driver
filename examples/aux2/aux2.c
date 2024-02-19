/*
 *
 * Copyright (c) [2020] by InvenSense, Inc.
 * 
 * Permission to use, copy, modify, and/or distribute this software for any
 * purpose with or without fee is hereby granted.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY
 * SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION
 * OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN
 * CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 *
 */

/* Driver */
#include "imu/inv_imu_driver_aux2.h"

/* Board drivers */
#include "system_interface.h"

/* std */
#include <stdio.h>

/*
 * This example showcases how to retrieve OIS data from AUX2 interface.
 * It gathers accelerometer and/or gyroscope data at expected 1kHz ODR as AUX2
 * does not have any INT pin capability.
 */

/*
 * Only SPI4 is available between SmartMotion and IMU as I2C would be too slow for AUX sensor ODR
 */
#define SERIF_TYPE UI_SPI4

/* Static variables */
static inv_imu_transport_t aux_dev; /* AUX2 Driver structure */

/* Static variables for command interface */
static uint8_t accel_en; /* Indicates accel state */
static uint8_t gyro_en; /* Indicates gyro state */

/* Static functions definition */
static int setup_mcu();
static int setup_imu();
static int configure_accel();
static int configure_gyro();
static int get_uart_command();
static int print_help();
static int print_current_config();

/* Main function implementation */
int main(void)
{
	int      rc = 0;
	uint64_t start;
	uint64_t prev;

	rc |= setup_mcu();
	SI_CHECK_RC(rc);

	INV_MSG(INV_MSG_LEVEL_INFO, "###");
	INV_MSG(INV_MSG_LEVEL_INFO, "### Example AUX2");
	INV_MSG(INV_MSG_LEVEL_INFO, "###");

	/* Reset commands interface states */
	accel_en = 1;
	gyro_en  = 1;

	rc |= setup_imu();
	SI_CHECK_RC(rc);

	/* Reset time related variable to know when to read from AUX2,
	which can not rely on INT1 when using virtual access to AUX port */
	start = si_get_time_us();
	prev  = start;

	do {
		uint64_t cur = si_get_time_us();
		/* Poll device for data */
		if ((cur - prev > 1000 /* 1kHz ODR on AUX2 */) && (accel_en || gyro_en)) {
			inv_imu_sensor_data_t d;
			char                  accel_str[40];
			char                  gyro_str[40];

			rc |= inv_imu_get_aux2_register_data(&aux_dev, &d);
			SI_CHECK_RC(rc);

			/*
			 * Convert data to SI units
			 * Accel and gyro data are coded as 16-bits signed (max_lsb = 2^(16-1) = 32768) with 
			 * the configured FSR (8 g and 2000 dps, see `setup_imu()` function).
			 */

			if (accel_en && d.accel_data[0] != INVALID_VALUE_FIFO &&
			    d.accel_data[1] != INVALID_VALUE_FIFO && d.accel_data[2] != INVALID_VALUE_FIFO) {
				float accel_g[3];
				accel_g[0] = (float)(d.accel_data[0] * 8 /* gee */) / 32768;
				accel_g[1] = (float)(d.accel_data[1] * 8 /* gee */) / 32768;
				accel_g[2] = (float)(d.accel_data[2] * 8 /* gee */) / 32768;
				snprintf(accel_str, 40, "Accel:% 8.2f % 8.2f % 8.2f g", accel_g[0], accel_g[1],
				         accel_g[2]);
			} else
				snprintf(accel_str, 40, "Accel:       -        -        -  ");

			if (gyro_en && d.gyro_data[0] != INVALID_VALUE_FIFO &&
			    d.gyro_data[1] != INVALID_VALUE_FIFO && d.gyro_data[2] != INVALID_VALUE_FIFO) {
				float gyro_dps[3];
				gyro_dps[0] = (float)(d.gyro_data[0] * 2000 /* dps */) / 32768;
				gyro_dps[1] = (float)(d.gyro_data[1] * 2000 /* dps */) / 32768;
				gyro_dps[2] = (float)(d.gyro_data[2] * 2000 /* dps */) / 32768;
				snprintf(gyro_str, 40, "Gyro:% 8.2f % 8.2f % 8.2f dps", gyro_dps[0], gyro_dps[1],
				         gyro_dps[2]);
			} else
				snprintf(gyro_str, 40, "Gyro:       -        -        -    ");

			INV_MSG(INV_MSG_LEVEL_INFO, "%10llu us   %s   %s", cur, accel_str, gyro_str);

			prev = cur;
		}

		rc |= get_uart_command();
	} while (rc == 0);

	return rc;
}

/* Initializes MCU peripherals. */
static int setup_mcu()
{
	int rc = 0;

	rc |= si_board_init();

	/* Configure UART for log */
	rc |= si_config_uart_for_print(SI_UART_ID_FTDI, INV_MSG_LEVEL_DEBUG);

	/* Init timer peripheral for sleep and get_time */
	rc |= si_init_timers();

	/* Initialize serial interface between MCU and IMU */
	rc |= si_io_imu_init(SERIF_TYPE);

	return rc;
}

/* Initializes IMU device and apply configuration. */
static int setup_imu()
{
	int     rc     = 0;
	uint8_t whoami = 0;

	/* Init transport layer */
	aux_dev.read_reg   = si_io_imu_read_reg;
	aux_dev.write_reg  = si_io_imu_write_reg;
	aux_dev.serif_type = SERIF_TYPE;

	/* Wait 3 ms to ensure device is properly supplied  */
	si_sleep_us(3000);

	/* Check whoami */
	rc |= inv_imu_read_reg(&aux_dev, WHO_AM_I, 1, &whoami);
	SI_CHECK_RC(rc);
	if (whoami != INV_IMU_WHOAMI) {
		INV_MSG(INV_MSG_LEVEL_ERROR, "Erroneous WHOAMI value.");
		INV_MSG(INV_MSG_LEVEL_ERROR, "  - Read 0x%02x", whoami);
		INV_MSG(INV_MSG_LEVEL_ERROR, "  - Expected 0x%02x", INV_IMU_WHOAMI);
		return -1;
	}

	/* Set FSR */
	rc |= inv_imu_set_aux2_accel_fsr(&aux_dev, FS_SEL_AUX_ACCEL_FS_SEL_8_G);
	rc |= inv_imu_set_aux2_gyro_fsr(&aux_dev, FS_SEL_AUX_GYRO_FS_SEL_2000_DPS);
	SI_CHECK_RC(rc);

	/* Set power modes */
	rc |= configure_accel();
	rc |= configure_gyro();
	SI_CHECK_RC(rc);

	return rc;
}

/* Configure power mode for accel */
static int configure_accel()
{
	int rc = 0;

	if (accel_en) {
		rc |= inv_imu_set_aux2_accel_mode(&aux_dev, INV_IMU_ENABLE);
		/* Discard samples during accel startup time */
		si_sleep_us(ACC_STARTUP_TIME_US);
	} else
		rc |= inv_imu_set_aux2_accel_mode(&aux_dev, INV_IMU_DISABLE);
	SI_CHECK_RC(rc);

	return 0;
}

/* Configure power mode for gyro */
static int configure_gyro()
{
	int rc = 0;

	if (gyro_en) {
		rc |= inv_imu_set_aux2_gyro_mode(&aux_dev, INV_IMU_ENABLE);
		/* Discard samples during gyro startup time */
		si_sleep_us(GYR_STARTUP_TIME_US);
	} else
		rc |= inv_imu_set_aux2_gyro_mode(&aux_dev, INV_IMU_DISABLE);
	SI_CHECK_RC(rc);

	return 0;
}

/* Get command from user through UART */
static int get_uart_command()
{
	int  rc  = 0;
	char cmd = 0;

	rc |= si_get_uart_command(SI_UART_ID_FTDI, &cmd);
	SI_CHECK_RC(rc);

	switch (cmd) {
	case 'a':
		accel_en = !accel_en;
		rc |= configure_accel();
		rc |= print_current_config();
		break;
	case 'g':
		gyro_en = !gyro_en;
		rc |= configure_gyro();
		rc |= print_current_config();
		break;
	case 'c':
		rc |= print_current_config();
		break;
	case 'h':
	case 'H':
		rc |= print_help();
		break;
	case 0:
		break; /* No command received */
	default:
		INV_MSG(INV_MSG_LEVEL_INFO, "Unknown command : %c", cmd);
		rc |= print_help();
		break;
	}

	return rc;
}

/* Help for UART command interface */
static int print_help()
{
	INV_MSG(INV_MSG_LEVEL_INFO, "#");
	INV_MSG(INV_MSG_LEVEL_INFO, "# Help");
	INV_MSG(INV_MSG_LEVEL_INFO, "#");
	INV_MSG(INV_MSG_LEVEL_INFO, "# 'a' : Toggle accel data");
	INV_MSG(INV_MSG_LEVEL_INFO, "# 'g' : Toggle gyro data");
	INV_MSG(INV_MSG_LEVEL_INFO, "# 'c' : Print current configuration");
	INV_MSG(INV_MSG_LEVEL_INFO, "# 'h' : Print this helper");
	INV_MSG(INV_MSG_LEVEL_INFO, "#");

	si_sleep_us(2000000); /* Give user some time to read */

	return 0;
}

/* Print current sample configuration */
static int print_current_config()
{
	INV_MSG(INV_MSG_LEVEL_INFO, "#");
	INV_MSG(INV_MSG_LEVEL_INFO, "# Current configuration");
	INV_MSG(INV_MSG_LEVEL_INFO, "#");
	INV_MSG(INV_MSG_LEVEL_INFO, "# Accel: %s", accel_en ? "Enabled" : "Disabled");
	INV_MSG(INV_MSG_LEVEL_INFO, "# Gyro: %s", gyro_en ? "Enabled" : "Disabled");
	INV_MSG(INV_MSG_LEVEL_INFO, "#");

	si_sleep_us(2000000); /* Give user some time to read */

	return 0;
}
