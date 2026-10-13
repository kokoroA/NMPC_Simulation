# SIL: 実機ファーム(Benzen_NMPC)をホスト上で動かす共有ライブラリ
#   2026-10-06: program/sil（Benzen_new 用）を複製し Benzen_NMPC 向けに改造したもの。
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
# -DFW_DIR=... で差し替え可（アブレーション用のファーム複製をビルドするため。2026-10-08）
if(NOT DEFINED FW_DIR)
  set(FW_DIR "/Users/kokoro/Documents/research_project/program/pico/pico-sdk2/pico-sdk/Benzen_NMPC"
    CACHE PATH "Benzen_NMPC firmware source directory")
endif()
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
  ${FW_DIR}/nmpc.cpp       # NMPC 本体（実機と同じ単精度・EIGEN_NO_MALLOC）
  ${FW_DIR}/nmpc_model.cpp
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
  # 実機 Benzen_NMPC/CMakeLists.txt と同じ定義（Wi-Fi テレメトリは無効＝NMPC_WIFI_TELEM 未定義）
  SYS_CLK_KHZ=125000
  DEBUG_SKIP_LOCK_COM=0
  NMPC_BENCH_MODE=0
  NMPC_BENCH_LOG=1
  NMPC_BENCH_COST=0
  NMPC_TR1_VERIFY=0
  NMPC_BENCH_SERIAL_LOG=1
  EIGEN_NO_MALLOC          # 実機と同じ制約を課す（動的確保があれば assert で落ちる）
  EIGEN_DONT_VECTORIZE     # 実機 Cortex-M33 に近い演算経路にする
)

target_compile_options(benzen_sil PRIVATE
  -Wno-unused-variable -Wno-unused-but-set-variable
  -Wno-write-strings -Wno-narrowing
)

find_package(Threads REQUIRED)
target_link_libraries(benzen_sil PRIVATE Threads::Threads)
