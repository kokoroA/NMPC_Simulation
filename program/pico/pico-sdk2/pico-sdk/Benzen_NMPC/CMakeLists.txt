cmake_minimum_required(VERSION 3.13)

set(PICO_BOARD pico2_w CACHE STRING "Board type" FORCE)
# NMPC warmup on core0: default 2KB stack overflows on 120x120 Hessian (→ main RAM .stack)
set(PICO_STACK_SIZE 0x8000)
# core1 stack lives in SCRATCH_X (4KB max on RP2350); nmpc_step は BSS scratch + 明示 GEMM で対応
set(PICO_CORE1_STACK_SIZE 0x1000)
add_compile_definitions(
  PICO_STACK_SIZE=${PICO_STACK_SIZE}
  PICO_CORE1_STACK_SIZE=${PICO_CORE1_STACK_SIZE}
)

include(pico_sdk_import.cmake)

project(pico_copter C CXX ASM)
set(CMAKE_C_STNDARD 11)
set(CMAKE_CXX_STANDARD 17)

set(PICO_EXAMPLES_PATH ${PROJECT_SOURCE_DIR})

pico_sdk_init()
# include_directories(/home/student01/pico/pico-sdk/eigen)
include_directories(/Users/kokoro/Documents/drone_project/pico/eigen)

add_executable(pico_copter
  pico_copter.cpp
  board_led.cpp
  sensor.cpp
  ekf.cpp
  lsm9ds1_reg.c
  pwm.cpp
  radio.cpp
  control.cpp
  nmpc.cpp
  nmpc_model.cpp
  WS2812.cpp
  rgbled.cpp
  i2c.cpp
  pid.cpp
  vl53l1_platform.c
  VL53L1X_api.c
  VL53L1X_calibration.c
  wifi_telemetry.cpp
  dhcpserver/dhcpserver.c
)

pico_generate_pio_header(pico_copter ${CMAKE_CURRENT_LIST_DIR}/WS2812.pio)


target_link_libraries(pico_copter 
  hardware_spi 
  hardware_pwm 
  hardware_uart 
  hardware_pio
  hardware_i2c
  hardware_clocks
  pico_stdlib
  pico_multicore
)

if (PICO_CYW43_SUPPORTED)
  target_link_libraries(pico_copter pico_cyw43_arch_lwip_threadsafe_background)
  target_include_directories(pico_copter PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
    ${CMAKE_CURRENT_LIST_DIR}/dhcpserver
  )
  pico_configure_ip4_address(pico_copter PRIVATE
    CYW43_DEFAULT_IP_AP_ADDRESS 192.168.4.1
  )
  target_compile_definitions(pico_copter PRIVATE NMPC_WIFI_TELEM=1 NMPC_BENCH_SERIAL_LOG=0)
else()
  target_compile_definitions(pico_copter PRIVATE NMPC_BENCH_SERIAL_LOG=1)
endif()

target_compile_definitions(pico_copter PRIVATE
  SYS_CLK_KHZ=125000
  DEBUG_SKIP_LOCK_COM=0
  # !! 値で ON/OFF するマクロは受け側を必ず #if で書くこと（#ifdef だと =0 でも真）
  NMPC_BENCH_MODE=0   # 1 = 机上検証挙動(スロットル利得 0.2 / Arm_flag==2 で常時 NMPC)
  NMPC_BENCH_LOG=1    # 1 = iter_us/loop_us 等のスナップショットとテレメトリ
  NMPC_BENCH_COST=0   # 1 = evaluate_cost() を毎周期実行(周期時間を食う)
  NMPC_TR1_VERIFY=0
  EIGEN_NO_MALLOC
)

pico_enable_stdio_usb(pico_copter 1)
pico_enable_stdio_uart(pico_copter 0)

pico_add_extra_outputs(pico_copter)
