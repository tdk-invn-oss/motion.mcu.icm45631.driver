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

/** @defgroup DriverAux2 AUX2 driver
 *  @brief Basic functions to drive AUX2 interfaces of the device
 *  @{
 */

/** @file inv_imu_driver_aux2.h */

#ifndef _INV_IMU_DRIVER_AUX2_H_
#define _INV_IMU_DRIVER_AUX2_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "imu/inv_imu_defs.h"

#if INV_IMU_AUX2_SUPPORTED

#include "imu/inv_imu_transport.h"

/** @brief Set accel power mode  for AUX2 interface.
 *  @param[in] t   Pointer to transport.
 *  @param[in] en  Requested power mode (INV_IMU_ENABLE or INV_IMU_DISABLE).
 *  @return        0 on success, negative value on error.
 */
int inv_imu_set_aux2_accel_mode(inv_imu_transport_t *t, uint8_t en);

/** @brief Set accel full scale range for the AUX2 interface.
 *  @param[in] t    Pointer to transport.
 *  @param[in] fsr  Requested full scale range.
 *  @return         0 on success, negative value on error.
 */
int inv_imu_set_aux2_accel_fsr(inv_imu_transport_t *t, fs_sel_aux_accel_fs_sel_t fsr);

/** @brief Get accel full scale range for the AUX2 interface.s
 *  @param[in] t     Pointer to transport.
 *  @param[out] fsr  Current full scale range for the AUX2 interface.
 *  @return          0 on success, negative value on error.
 */
int inv_imu_get_aux2_accel_fsr(inv_imu_transport_t *t, fs_sel_aux_accel_fs_sel_t *fsr);

/** @brief Set gyro power mode for AUX2 interface.
 *  @param[in] t   Pointer to transport.
 *  @param[in] en  Requested power mode (INV_IMU_ENABLE or INV_IMU_DISABLE).
 *  @return        0 on success, negative value on error.
 */
int inv_imu_set_aux2_gyro_mode(inv_imu_transport_t *t, uint8_t en);

/** @brief Set gyro full scale range for the AUX2 interface.
 *  @param[in] t    Pointer to transport.
 *  @param[in] fsr  Requested full scale range.
 *  @return         0 on success, negative value on error.
 */
int inv_imu_set_aux2_gyro_fsr(inv_imu_transport_t *t, fs_sel_aux_gyro_fs_sel_t fsr);

/** @brief Get gyro full scale range for the AUX2 interface.
 *  @param[in] t     Pointer to transport.
 *  @param[out] fsr  Current full scale range for the AUX2 interface.
 *  @return          0 on success, negative value on error.
 */
int inv_imu_get_aux2_gyro_fsr(inv_imu_transport_t *t, fs_sel_aux_gyro_fs_sel_t *fsr);

/** @brief Get AUX2 interface register data.
 *  @param[in] t             Pointer to transport.
 *  @param[out] sensor_data  Current accel, gyro and temperature data from the AUX2 data registers.
 *  @return                  0 on success, negative value on error.
 */
int inv_imu_get_aux2_register_data(inv_imu_transport_t *t, inv_imu_sensor_data_t *sensor_data);

#endif /* INV_IMU_AUX2_SUPPORTED */

#ifdef __cplusplus
}
#endif

#endif /* _INV_IMU_DRIVER_AUX2_H_ */

/** @} */
