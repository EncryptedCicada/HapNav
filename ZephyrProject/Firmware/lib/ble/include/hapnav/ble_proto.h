/*
 * HapNav BLE protocol — shared between pin (central) and wristband (peripheral).
 *
 * One GATT service, one writable characteristic. Every sample period the pin
 * fuses its IMU + mag into a quaternion, grabs the latest VL53L5CX 8x8 ToF
 * frame, and writes the combined frame (write-without-response) to the
 * wristband. The wristband prints it as JSON.
 */
#ifndef HAPNAV_BLE_PROTO_H_
#define HAPNAV_BLE_PROTO_H_

#include <stdint.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/toolchain.h>

/* 128-bit UUID base: 5a9c-XXXX-7e6f-4abc-9c12-aabbccdd1100 */
#define HAPNAV_UUID_VAL(short16) \
	BT_UUID_128_ENCODE(0x5a9c0000 | (short16), \
			   0x7e6f, 0x4abc, 0x9c12, 0xaabbccdd1100)

#define HAPNAV_UUID_SVC_VAL          HAPNAV_UUID_VAL(0x0001)
#define HAPNAV_UUID_FRAME_VAL        HAPNAV_UUID_VAL(0x0004)

#define HAPNAV_TOF_ZONES   64   /* 8x8 grid */

/*
 * Combined per-sample frame, 212 bytes packed.
 * `timestamp_ms` is the pin's k_uptime_get_32() at frame assembly; kept for
 * latency debugging (not printed).
 * `quat` is unit quaternion [w, x, y, z] from Madgwick AHRS on the pin.
 * `distances_mm` and `target_status` are VL53L5CX ULD outputs at 8x8 resolution.
 */
struct hapnav_frame {
	uint32_t timestamp_ms;                  /*   4  */
	float    quat[4];                       /*  16  */
	int16_t  distances_mm[HAPNAV_TOF_ZONES]; /* 128 */
	uint8_t  target_status[HAPNAV_TOF_ZONES];/*  64 */
} __packed;

#define HAPNAV_FRAME_SIZE  sizeof(struct hapnav_frame)

/* Conversion constants — used by whoever wants engineering units. */
#define HAPNAV_ACCEL_MG_PER_LSB     0.061f
#define HAPNAV_GYRO_MDPS_PER_LSB    70.0f
#define HAPNAV_MAG_MGAUSS_PER_LSB   1.5f

#endif /* HAPNAV_BLE_PROTO_H_ */
