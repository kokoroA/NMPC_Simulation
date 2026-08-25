// SIL 版 sensor.cpp
//
// 実機の sensor.cpp は LSM9DS1 を SPI で、VL53L1X を I2C で叩く。
// SIL では「関数レベル」で差し替え、Python 側が計算したセンサ生値を
// そのままグローバル配列へ注入する。
//
// ⚠ 単位と符号は実機ドライバの出力と厳密に一致させること。
//    control.cpp の sensor_read() が下記の変換を掛けるため:
//      Ax = -acceleration_mg[0] * GRAV * 0.001    (X 反転, milli-g)
//      Ay = -acceleration_mg[1] * GRAV * 0.001    (Y 反転, milli-g)
//      Az =  acceleration_mg[2] * GRAV * 0.001    (Z そのまま)
//      Wp =  angular_rate_mdps[0] * pi/180/1000   (milli-deg/s)
//      Wq =  angular_rate_mdps[1] * pi/180/1000
//      Wr = -angular_rate_mdps[2] * pi/180/1000   (Z 反転)
//      Mx0 = -magnetic_field_mgauss[0]            (milli-gauss)
//      My0 =  magnetic_field_mgauss[1]
//      Mz0 = -magnetic_field_mgauss[2]
#include "sensor.hpp"

#include <atomic>
#include <cstring>
#include <mutex>

// --- 実機 sensor.cpp と同じグローバル ------------------------------------
float acceleration_mg[3]      = {0.0f, 0.0f, 0.0f};
float angular_rate_mdps[3]    = {0.0f, 0.0f, 0.0f};
float magnetic_field_mgauss[3] = {0.0f, 0.0f, 0.0f};

int8_t   Status = 0;
int16_t  OffsetValue = 0;
uint16_t dev = 0x52;
uint16_t distance = 0;
uint8_t  isDataReady = 0;
// Temp / Temp2 / IntPol は control.cpp が定義するためここでは定義しない
uint8_t  data_count = 0;
uint8_t  rangeStatus = 0;
uint8_t  state = 0;
uint8_t  tmp = 0;
uint8_t  Kalman_distance = 0;
int      sleep_time = 0;
int      ms = 0;

// --- 注入バッファ ---------------------------------------------------------
// core1(100Hz) と ホストスレッドの競合を避けるため二重化する。
namespace {
std::mutex g_mtx;
float g_acc[3]  = {0.0f, 0.0f, 1000.0f};  // 静止時 1g（Z 正）
float g_gyr[3]  = {0.0f, 0.0f, 0.0f};
float g_mag[3]  = {300.0f, 0.0f, 400.0f};
uint16_t g_range_mm = 0;
}

// ホスト（Python）から呼ぶ注入 API
extern "C" void sil_sensor_set_imu(const float acc_mg[3],
                                   const float gyr_mdps[3],
                                   const float mag_mgauss[3]) {
    std::lock_guard<std::mutex> lk(g_mtx);
    std::memcpy(g_acc, acc_mg, sizeof(g_acc));
    std::memcpy(g_gyr, gyr_mdps, sizeof(g_gyr));
    std::memcpy(g_mag, mag_mgauss, sizeof(g_mag));
}

extern "C" void sil_sensor_set_range_mm(uint16_t mm) {
    std::lock_guard<std::mutex> lk(g_mtx);
    g_range_mm = mm;
}

// --- 実機と同じシグネチャ -------------------------------------------------
void imu_mag_init(void) {
    // 実機では WHO_AM_I 確認・ODR/レンジ設定・リセット待ちを行う。
    // SIL では何もしない（センサ設定ミスは再現できない = 既知の限界）。
}

void imu_mag_data_read(void) {
    std::lock_guard<std::mutex> lk(g_mtx);
    std::memcpy(acceleration_mg, g_acc, sizeof(acceleration_mg));
    std::memcpy(angular_rate_mdps, g_gyr, sizeof(angular_rate_mdps));
    std::memcpy(magnetic_field_mgauss, g_mag, sizeof(magnetic_field_mgauss));
}

void initialize_Altitude(void) {
    Status = 0;
    isDataReady = 1;
    rangeStatus = 0;
}

void get_Altitude(void) {
    std::lock_guard<std::mutex> lk(g_mtx);
    distance = g_range_mm;
    isDataReady = 1;
    rangeStatus = 0;
}
