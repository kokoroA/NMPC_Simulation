#include "pico_copter.hpp"
#include "wifi_telemetry.hpp"

//グローバル変数
uint8_t Arm_flag=0;
uint8_t Red_flag = 0;
semaphore_t sem;


int main(void)
{
  int start_wait=5;

  //Initialize stdio for Pico
  stdio_init_all();

  // RP2350-E9: force PWM pins low and disable input buffer before other init
  pwm_pins_early_init();

  printf("#start setup\r\n");

  #ifdef NMPC_WIFI_TELEM
  if (wifi_telemetry_init() != 0)
    printf("#wifi init failed\r\n");
#endif

  int led_rc = board_led_init();
  if (led_rc != PICO_OK)
    printf("#board led init failed\r\n");
  printf("#init pico\r\n");

  //Initialize LSM9DS1
  imu_mag_init();
  printf("#init gyro\r\n");

  //Initialize Radio
  radio_init();
  printf("#init radio\r\n");

  //Initialize Variavle
  variable_init();
  printf("#init cariavle\r\n");

  //Initilize Control
  control_init();
  // Kalman_init();
  // initialize_Altitude();
  // printf("#init control\r\n");

  // //RGB LED off
  rgbled_off();
  
  //Initialize PWM
  //Start 400Hz Interval
  ESC_calib= 0;
  pwm_init();
  printf("#init pwm\r\n");

  while(start_wait)
  {
    start_wait--;
    printf("#Please wait %d[s]\r",start_wait); 
    sleep_ms(1000);
  }
  printf("\n");
 
  //マルチコア関連の設定
  sem_init(&sem, 0, 1);
  multicore_launch_core1(angle_control);  

  Arm_flag=1;

  uint32_t last_bench_print = time_us_32();
  
  while(1)
  {
    board_led_poll();
    debug_poll_pending_messages();
    // NMPC ベンチ (10Hz): シリアル CSV と/または Wi-Fi テレメ。NMPC_BENCH_LOG 未定義時は no-op。
    if (time_us_32() - last_bench_print > 100000)
    {
      last_bench_print = time_us_32();
      debug_print_nmpc_bench();
    }
    tight_loop_contents();
    while (Logoutputflag==1){
      log_output();
    }
  }

  return 0;
}
