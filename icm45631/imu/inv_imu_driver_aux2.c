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

#include "imu/inv_imu_driver_aux2.h"

#if INV_IMU_AUX2_SUPPORTED

int inv_imu_set_aux2_accel_mode(inv_imu_transport_t *t, uint8_t en)
{
	int             status = INV_IMU_OK;
	pwr_mgmt_aux2_t pwr_mgmt_aux2;

	status |= inv_imu_read_reg(t, PWR_MGMT_AUX2, 1, (uint8_t *)&pwr_mgmt_aux2);
	pwr_mgmt_aux2.accel_aux2_en = en;
	status |= inv_imu_write_reg(t, PWR_MGMT_AUX2, 1, (uint8_t *)&pwr_mgmt_aux2);

	return status;
}

int inv_imu_set_aux2_accel_fsr(inv_imu_transport_t *t, fs_sel_aux_accel_fs_sel_t fsr)
{
	int           status = INV_IMU_OK;
	fs_sel_aux2_t fs_sel_aux2;

	status |= inv_imu_read_reg(t, FS_SEL_AUX2, 1, (uint8_t *)&fs_sel_aux2);
	fs_sel_aux2.accel_aux2_fs_sel = fsr;
	status |= inv_imu_write_reg(t, FS_SEL_AUX2, 1, (uint8_t *)&fs_sel_aux2);

	return status;
}

int inv_imu_get_aux2_accel_fsr(inv_imu_transport_t *t, fs_sel_aux_accel_fs_sel_t *fsr)
{
	int           status = INV_IMU_OK;
	fs_sel_aux2_t fs_sel_aux2;

	status |= inv_imu_read_reg(t, FS_SEL_AUX2, 1, (uint8_t *)&fs_sel_aux2);
	*fsr = (fs_sel_aux_accel_fs_sel_t)fs_sel_aux2.accel_aux2_fs_sel;

	return status;
}

int inv_imu_set_aux2_gyro_mode(inv_imu_transport_t *t, uint8_t en)
{
	int             status = INV_IMU_OK;
	pwr_mgmt_aux2_t pwr_mgmt_aux2;

	status |= inv_imu_read_reg(t, PWR_MGMT_AUX2, 1, (uint8_t *)&pwr_mgmt_aux2);
	pwr_mgmt_aux2.gyro_aux2_en = en;
	status |= inv_imu_write_reg(t, PWR_MGMT_AUX2, 1, (uint8_t *)&pwr_mgmt_aux2);

	return status;
}

int inv_imu_set_aux2_gyro_fsr(inv_imu_transport_t *t, fs_sel_aux_gyro_fs_sel_t fsr)
{
	int           status = INV_IMU_OK;
	fs_sel_aux2_t fs_sel_aux2;

	status |= inv_imu_read_reg(t, FS_SEL_AUX2, 1, (uint8_t *)&fs_sel_aux2);
	fs_sel_aux2.gyro_aux2_fs_sel = fsr;
	status |= inv_imu_write_reg(t, FS_SEL_AUX2, 1, (uint8_t *)&fs_sel_aux2);

	return status;
}

int inv_imu_get_aux2_gyro_fsr(inv_imu_transport_t *t, fs_sel_aux_gyro_fs_sel_t *fsr)
{
	int           status = INV_IMU_OK;
	fs_sel_aux2_t fs_sel_aux2;

	status |= inv_imu_read_reg(t, FS_SEL_AUX2, 1, (uint8_t *)&fs_sel_aux2);
	*fsr = (fs_sel_aux_gyro_fs_sel_t)fs_sel_aux2.gyro_aux2_fs_sel;

	return status;
}

int inv_imu_get_aux2_register_data(inv_imu_transport_t *t, inv_imu_sensor_data_t *sensor_data)
{
	int status = INV_IMU_OK;

	status |= inv_imu_read_reg(t, ACCEL_DATA_X1_AUX2, sizeof(inv_imu_sensor_data_t),
	                           (uint8_t *)sensor_data);

	return status;
}

#endif
