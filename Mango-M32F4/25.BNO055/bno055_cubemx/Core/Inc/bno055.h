/**
 * @file    bno055.h
 * @brief   BNO055 9-axis IMU driver for STM32F4 HAL (I2C)
 *
 * 지원 기능:
 *  - NDOF Fusion 모드 (Euler angle, Quaternion)
 *  - Raw sensor 데이터 (Accelerometer, Gyroscope, Magnetometer)
 *  - Calibration 상태 확인 및 저장/복원
 *  - 내부/외부 클럭 선택
 *
 * 데이터시트: BST-BNO055-DS000-18 Rev 1.8 (October 2021)
 */

#ifndef BNO055_H
#define BNO055_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

/* ─────────────────────────────────────────────
 * I2C 주소
 * COM3 핀 High → 0x29 (default)
 * COM3 핀 Low  → 0x28
 * ─────────────────────────────────────────────*/
#define BNO055_I2C_ADDR_HIGH    (0x29 << 1)   /* COM3 = HIGH (default) */
#define BNO055_I2C_ADDR_LOW     (0x28 << 1)   /* COM3 = LOW            */

/* ─────────────────────────────────────────────
 * 레지스터 맵 (Page 0)
 * ─────────────────────────────────────────────*/
#define BNO055_CHIP_ID_REG      0x00  /* 고정값: 0xA0 */
#define BNO055_ACC_CHIP_ID      0x01  /* 0xFB */
#define BNO055_MAG_CHIP_ID      0x02  /* 0x32 */
#define BNO055_GYR_CHIP_ID      0x03  /* 0x0F */

/* 가속도 Raw (6 bytes: 0x08~0x0D) */
#define BNO055_ACC_DATA_X_LSB   0x08

/* 자이로 Raw (6 bytes: 0x14~0x19) */
#define BNO055_GYR_DATA_X_LSB   0x14

/* 오일러각 (6 bytes: 0x1A~0x1F)  단위: 1/16 degree */
#define BNO055_EUL_DATA_X_LSB   0x1A  /* Heading(Yaw)  */
#define BNO055_EUL_DATA_Y_LSB   0x1C  /* Roll          */
#define BNO055_EUL_DATA_Z_LSB   0x1E  /* Pitch         */

/* 쿼터니언 (8 bytes: 0x20~0x27)  단위: 1/2^14 */
#define BNO055_QUA_DATA_W_LSB   0x20

/* 선형가속도 (중력 제거, 6 bytes: 0x28~0x2D) */
#define BNO055_LIA_DATA_X_LSB   0x28

/* 중력벡터 (6 bytes: 0x2E~0x33) */
#define BNO055_GRV_DATA_X_LSB   0x2E

/* 온도 */
#define BNO055_TEMP_REG         0x34

/* 상태 레지스터 */
#define BNO055_CALIB_STAT_REG   0x35  /* Calibration status */
#define BNO055_ST_RESULT_REG    0x36  /* Self-test result   */
#define BNO055_INT_STATUS_REG   0x37
#define BNO055_SYS_STATUS_REG   0x39
#define BNO055_SYS_ERR_REG      0x3A

/* 설정 레지스터 */
#define BNO055_UNIT_SEL_REG     0x3B
#define BNO055_OPR_MODE_REG     0x3D
#define BNO055_PWR_MODE_REG     0x3E
#define BNO055_SYS_TRIGGER_REG  0x3F
#define BNO055_TEMP_SOURCE_REG  0x40
#define BNO055_AXIS_MAP_CONFIG  0x41
#define BNO055_AXIS_MAP_SIGN    0x42

/* 캘리브레이션 오프셋 (Page 0: 0x55~0x6A) */
#define BNO055_ACC_OFFSET_X_LSB 0x55
#define BNO055_MAG_OFFSET_X_LSB 0x5B
#define BNO055_GYR_OFFSET_X_LSB 0x61
#define BNO055_ACC_RADIUS_LSB   0x67
#define BNO055_MAG_RADIUS_LSB   0x69

/* Page 선택 */
#define BNO055_PAGE_ID_REG      0x07

/* ─────────────────────────────────────────────
 * 동작 모드 (OPR_MODE 레지스터)
 * ─────────────────────────────────────────────*/
typedef enum {
    BNO055_MODE_CONFIG      = 0x00,  /* 설정 전용 (기본값, 전원 ON 시) */
    /* Non-Fusion 모드 */
    BNO055_MODE_ACCONLY     = 0x01,
    BNO055_MODE_MAGONLY     = 0x02,
    BNO055_MODE_GYROONLY    = 0x03,
    BNO055_MODE_ACCMAG      = 0x04,
    BNO055_MODE_ACCGYRO     = 0x05,
    BNO055_MODE_MAGGYRO     = 0x06,
    BNO055_MODE_AMG         = 0x07,
    /* Fusion 모드 */
    BNO055_MODE_IMU         = 0x08,  /* ACC + GYR (Mag 미사용, 빠른 응답) */
    BNO055_MODE_COMPASS     = 0x09,
    BNO055_MODE_M4G         = 0x0A,
    BNO055_MODE_NDOF_FMC_OFF = 0x0B,
    BNO055_MODE_NDOF        = 0x0C,  /* 9축 완전 Fusion (권장) */
} BNO055_OpMode_t;

/* ─────────────────────────────────────────────
 * 전원 모드
 * ─────────────────────────────────────────────*/
typedef enum {
    BNO055_POWER_NORMAL     = 0x00,
    BNO055_POWER_LOW        = 0x01,
    BNO055_POWER_SUSPEND    = 0x02,
} BNO055_PowerMode_t;

/* ─────────────────────────────────────────────
 * 오일러각 구조체
 * heading: 0 ~ 360도  (North = 0)
 * roll:  -90 ~ +90도
 * pitch: -180 ~ +180도
 * ─────────────────────────────────────────────*/
typedef struct {
    float heading;  /* Yaw   (Z축 회전) */
    float roll;     /* Y축 회전         */
    float pitch;    /* X축 회전         */
} BNO055_Euler_t;

/* ─────────────────────────────────────────────
 * 쿼터니언 구조체
 * ─────────────────────────────────────────────*/
typedef struct {
    float w, x, y, z;
} BNO055_Quaternion_t;

/* ─────────────────────────────────────────────
 * 3축 벡터 구조체 (가속도, 자이로, 자력계)
 * ─────────────────────────────────────────────*/
typedef struct {
    float x, y, z;
} BNO055_Vec3_t;

/* ─────────────────────────────────────────────
 * 캘리브레이션 상태
 * 0 = 미보정, 3 = 완전 보정
 * ─────────────────────────────────────────────*/
typedef struct {
    uint8_t sys;  /* System  (0~3) */
    uint8_t gyro; /* Gyro    (0~3) */
    uint8_t accel;/* Accel   (0~3) */
    uint8_t mag;  /* Mag     (0~3) */
} BNO055_CalibStatus_t;

/* ─────────────────────────────────────────────
 * 캘리브레이션 오프셋 프로파일 (EEPROM 저장용)
 * ─────────────────────────────────────────────*/
typedef struct {
    int16_t accel_offset[3];  /* X, Y, Z */
    int16_t mag_offset[3];
    int16_t gyro_offset[3];
    int16_t accel_radius;
    int16_t mag_radius;
} BNO055_CalibData_t;

/* ─────────────────────────────────────────────
 * 드라이버 핸들 구조체
 * ─────────────────────────────────────────────*/
typedef struct {
    I2C_HandleTypeDef *hi2c;      /* STM32 HAL I2C 핸들 */
    uint8_t           dev_addr;   /* 7-bit 주소 << 1     */
    BNO055_OpMode_t   cur_mode;   /* 현재 동작 모드       */
    bool              use_ext_crystal; /* 외부 크리스탈 사용 여부 */
} BNO055_Handle_t;

/* ─────────────────────────────────────────────
 * 반환 코드
 * ─────────────────────────────────────────────*/
typedef enum {
    BNO055_OK           = 0,
    BNO055_ERR_I2C      = -1,
    BNO055_ERR_CHIP_ID  = -2,
    BNO055_ERR_TIMEOUT  = -3,
    BNO055_ERR_MODE     = -4,
} BNO055_Status_t;

/* ─────────────────────────────────────────────
 * API 함수 선언
 * ─────────────────────────────────────────────*/

/* 초기화 */
BNO055_Status_t BNO055_Init(BNO055_Handle_t *hbno,
                             I2C_HandleTypeDef *hi2c,
                             uint8_t i2c_addr,
                             bool use_ext_crystal);

/* 동작 모드 설정 */
BNO055_Status_t BNO055_SetMode(BNO055_Handle_t *hbno, BNO055_OpMode_t mode);

/* 데이터 읽기 */
BNO055_Status_t BNO055_GetEuler(BNO055_Handle_t *hbno, BNO055_Euler_t *euler);
BNO055_Status_t BNO055_GetQuaternion(BNO055_Handle_t *hbno, BNO055_Quaternion_t *quat);
BNO055_Status_t BNO055_GetAccel(BNO055_Handle_t *hbno, BNO055_Vec3_t *accel);
BNO055_Status_t BNO055_GetGyro(BNO055_Handle_t *hbno, BNO055_Vec3_t *gyro);
BNO055_Status_t BNO055_GetLinearAccel(BNO055_Handle_t *hbno, BNO055_Vec3_t *lia);
BNO055_Status_t BNO055_GetTemperature(BNO055_Handle_t *hbno, int8_t *temp_c);

/* 캘리브레이션 */
BNO055_Status_t BNO055_GetCalibStatus(BNO055_Handle_t *hbno, BNO055_CalibStatus_t *cal);
bool            BNO055_IsFullyCalibrated(BNO055_Handle_t *hbno);
BNO055_Status_t BNO055_GetCalibData(BNO055_Handle_t *hbno, BNO055_CalibData_t *data);
BNO055_Status_t BNO055_SetCalibData(BNO055_Handle_t *hbno, const BNO055_CalibData_t *data);

/* 시스템 상태 */
BNO055_Status_t BNO055_GetSysStatus(BNO055_Handle_t *hbno,
                                     uint8_t *sys_status,
                                     uint8_t *sys_error,
                                     uint8_t *st_result);
BNO055_Status_t BNO055_Reset(BNO055_Handle_t *hbno);

#ifdef __cplusplus
}
#endif

#endif /* BNO055_H */
