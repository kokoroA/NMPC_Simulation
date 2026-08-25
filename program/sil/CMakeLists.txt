# SIL: 実機ファーム(Benzen_new)をホスト上で動かす共有ライブラリ
#
# 設計方針:
#   - 制御ロジック(control/pid/ekf/auto_maneuvers/pwm/radio)は実機ソースをそのまま使う
#   - pico-sdk は shim/ が肩代わりする（include パスで本物より先に解決させる）
#   - sensor.cpp だけは hal_sil/sensor_sil.cpp に差し替える
cmake_minimum_required(VERSION 3.16)
project(benzen_sil CXX C)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# --- ファームのソース位置 -------------------------------------------------
set(FW_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../pico/pico-sdk2/pico-sdk/Benzen_new"
    CACHE PATH "Benzen_new firmware source directory")
if(NOT EXISTS "${FW_DIR}/control.cpp")
  message(FATAL_ERROR "ファームが見つかりません: ${FW_DIR}")
endif()

# --- Eigen ----------------------------------------------------------------
# 実機 CMakeLists は別プロジェクトの絶対パスをハードコードしているため、
# ホストではシステム Eigen を使う（見つからなければ EIGEN_DIR で指定）。
find_path(EIGEN_INCLUDE_DIR Eigen/Dense
  HINTS ${EIGEN_DIR} /usr/local/include/eigen3 /opt/homebrew/include/eigen3 /usr/include/eigen3)
if(NOT EIGEN_INCLUDE_DIR)
  message(FATAL_ERROR "Eigen が見つかりません。-DEIGEN_DIR=<path> で指定してください")
endif()
message(STATUS "Eigen: ${EIGEN_INCLUDE_DIR}")

# --- ソース ---------------------------------------------------------------
set(FW_SOURCES
  ${FW_DIR}/control.cpp
  ${FW_DIR}/pid.cpp
  ${FW_DIR}/ekf.cpp
  ${FW_DIR}/auto_maneuvers.cpp
  ${FW_DIR}/pwm.cpp        # set_duty_* のクリップ処理を本物のまま使う
  ${FW_DIR}/radio.cpp      # Chdata[] の定義を持つ
  ${FW_DIR}/rgbled.cpp
  ${FW_DIR}/WS2812.cpp
  ${FW_DIR}/i2c.cpp
  ${FW_DIR}/board_led.cpp
)

set(SIL_SOURCES
  shim/shim.cpp
  hal_sil/sensor_sil.cpp
  api/sil_api.cpp
)

add_library(benzen_sil SHARED ${FW_SOURCES} ${SIL_SOURCES})

# shim を最優先で解決させる
target_include_directories(benzen_sil BEFORE PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/shim
)
target_include_directories(benzen_sil PRIVATE
  ${FW_DIR}
  ${EIGEN_INCLUDE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/api
)

target_compile_definitions(benzen_sil PRIVATE
  SIL_BUILD=1
  EIGEN_NO_MALLOC          # 実機と同じ制約を課す（動的確保があれば assert で落ちる）
  EIGEN_DONT_VECTORIZE     # 実機 Cortex-M33 に近い演算経路にする
)

target_compile_options(benzen_sil PRIVATE
  -Wno-unused-variable -Wno-unused-but-set-variable
  -Wno-write-strings -Wno-narrowing
)

find_package(Threads REQUIRED)
target_link_libraries(benzen_sil PRIVATE Threads::Threads)
