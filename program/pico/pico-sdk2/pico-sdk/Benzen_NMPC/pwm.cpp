/*
 * pwm.cpp
*/
#include "pwm.hpp"

//グローバル変数
uint8_t ESC_calib=0;

//ファイル内グローバル変数
// RP2040 PWM slice: GP2/3=slice1, GP20/21=slice2, GP14/15=slice7
const uint Slice_num_rear=1;    // GP2 (MR), GP3 (ML)
const uint Slice_num_middle=2;  // GP20 (FL), GP21 (RL)
const uint Slice_num_front=7;   // GP14 (FR), GP15 (RR)
const uint Slice_num_servo = 3;

// RP2350-E9 workaround: motor/servo PWM pins (GP2/3/6/14/15/20/21)
static const uint pwm_pins[] = {2, 3, 6, 14, 15, 20, 21};
static const uint pwm_pin_count = sizeof(pwm_pins) / sizeof(pwm_pins[0]);

static void pwm_pins_disable_input(void)
{
    for (uint i = 0; i < pwm_pin_count; i++) {
        gpio_set_input_enabled(pwm_pins[i], false);
    }
}

void pwm_pins_early_init(void)
{
    for (uint i = 0; i < pwm_pin_count; i++) {
        gpio_init(pwm_pins[i]);
        gpio_set_dir(pwm_pins[i], GPIO_OUT);
        gpio_put(pwm_pins[i], 0);
        gpio_set_input_enabled(pwm_pins[i], false);
    }
}

void pwm_init()
{
    // PWMの設定
    // Tell GPIO 2-6 they are allocated to the PWM for Motor & Servo Control
    gpio_set_function(2, GPIO_FUNC_PWM);//Middle Right (MR) Motor PWM
    gpio_set_function(3, GPIO_FUNC_PWM);//Middle Left  (ML) Motor PWM
    gpio_set_function(14, GPIO_FUNC_PWM);//Front Right (FR) Motor PWM
    gpio_set_function(15, GPIO_FUNC_PWM);//Rear  Right (RR) Motor PWM
    gpio_set_function(20, GPIO_FUNC_PWM);//Front Left  (FL) Motor PWM
    gpio_set_function(21, GPIO_FUNC_PWM);//Rear  Left  (RL) Motor PWM
    gpio_set_function(6, GPIO_FUNC_PWM);//Servo PWM
    pwm_pins_disable_input();

    // Find out which PWM slice is connected to GPIO 0 (it's slice 0)
    // Set period T
    // T=(wrap+1)*clkdiv/sysclock
    // T=(24999+1)*100/125e6=25000e2/125e6=200e-4=0.02s(=50Hz)
    pwm_set_wrap(Slice_num_front, 3124);
    pwm_set_wrap(Slice_num_middle,3124);
    pwm_set_wrap(Slice_num_rear,  3124);
    pwm_set_wrap(Slice_num_servo, 3124);

    pwm_set_clkdiv(Slice_num_front, 100.0);
    pwm_set_clkdiv(Slice_num_middle, 100.0);
    pwm_set_clkdiv(Slice_num_rear, 100.0);
    pwm_set_clkdiv(Slice_num_servo, 100.0);

    pwm_clear_irq(Slice_num_front);
    pwm_set_irq_enabled(Slice_num_front, true);
    irq_set_exclusive_handler(PWM_IRQ_WRAP, MAINLOOP);
    irq_set_enabled(PWM_IRQ_WRAP, true);

    if(ESC_calib==1)
    {
      // ESC calibration
      // Set Duty
      // DutyA=clkdiv*PWM_CHAN_A/sysclock
      pwm_set_chan_level(Slice_num_front, PWM_CHAN_A, DUTYMAX);
      pwm_set_chan_level(Slice_num_middle, PWM_CHAN_A, DUTYMAX);
      pwm_set_chan_level(Slice_num_rear,  PWM_CHAN_A, DUTYMAX);
      pwm_set_chan_level(Slice_num_front, PWM_CHAN_B, DUTYMAX);
      pwm_set_chan_level(Slice_num_middle,  PWM_CHAN_B, DUTYMAX);
      pwm_set_chan_level(Slice_num_rear,  PWM_CHAN_B, DUTYMAX);
    } 
    else
    {
      pwm_set_chan_level(Slice_num_front, PWM_CHAN_A, DUTYMIN);
      pwm_set_chan_level(Slice_num_middle, PWM_CHAN_A, DUTYMIN);
      pwm_set_chan_level(Slice_num_rear,  PWM_CHAN_A, DUTYMIN);
      pwm_set_chan_level(Slice_num_front, PWM_CHAN_B, DUTYMIN);
      pwm_set_chan_level(Slice_num_middle,  PWM_CHAN_B, DUTYMIN);
      pwm_set_chan_level(Slice_num_rear,  PWM_CHAN_B, DUTYMIN);
    }

    // Set the PWM running
    //sleep_ms(500);
    pwm_set_enabled(Slice_num_front, true);
    pwm_set_enabled(Slice_num_middle, true);
    pwm_set_enabled(Slice_num_rear,  true);
    pwm_set_enabled(Slice_num_servo, true);

    sleep_ms(3000);
    
    pwm_set_chan_level(Slice_num_front, PWM_CHAN_A, DUTYMIN);
    pwm_set_chan_level(Slice_num_front, PWM_CHAN_B, DUTYMIN);
    pwm_set_chan_level(Slice_num_middle, PWM_CHAN_A, DUTYMIN);
    pwm_set_chan_level(Slice_num_middle, PWM_CHAN_B, DUTYMIN);
    pwm_set_chan_level(Slice_num_rear,  PWM_CHAN_A, DUTYMIN);
    pwm_set_chan_level(Slice_num_rear,  PWM_CHAN_B, DUTYMIN);
    // pwm_set_chan_level(Slice_num_rear,  PWM_CHAN_B, DUTYMIN);
    pwm_set_chan_level(Slice_num_servo, PWM_CHAN_A, 625);


    sleep_ms(1000);
}


void set_duty_rr(float duty)
{
    if (duty>0.97)duty=0.97;  
    if (duty<0.01)duty=0.01;
    duty=(float)(DUTYMAX-DUTYMIN)*duty+DUTYMIN;
    if (duty>DUTYMAX)duty=DUTYMAX;
    if (duty<DUTYMIN)duty=DUTYMIN;
    // pwm_set_chan_level(Slice_num_front, PWM_CHAN_B, duty);  // GP15
    pwm_set_chan_level(Slice_num_front, PWM_CHAN_A, duty);  // GP15
    //printf("%4.0f ", duty);
}

void set_duty_fr(float duty)
{
    if (duty>0.97)duty=0.97;  
    if (duty<0.01)duty=0.01;
    duty=(float)(DUTYMAX-DUTYMIN)*duty+DUTYMIN;
    if (duty>DUTYMAX)duty=DUTYMAX;
    if (duty<DUTYMIN)duty=DUTYMIN;
    // pwm_set_chan_level(Slice_num_front, PWM_CHAN_A, duty);  // GP14
    pwm_set_chan_level(Slice_num_front, PWM_CHAN_B, duty);  // GP14
    //printf("%4.0f ", duty);
}

void set_duty_rl(float duty)
{
    if (duty>0.97)duty=0.97;  
    if (duty<0.01)duty=0.01;
    duty=(float)(DUTYMAX-DUTYMIN)*duty+DUTYMIN;
    if (duty>DUTYMAX)duty=DUTYMAX;
    if (duty<DUTYMIN)duty=DUTYMIN;
    // pwm_set_chan_level(Slice_num_middle, PWM_CHAN_B, duty);  // GP21
    pwm_set_chan_level(Slice_num_rear, PWM_CHAN_A, duty);  // GP21
    //printf("%4.0f ", duty);
}

void set_duty_fl(float duty)
{
    if (duty>0.97)duty=0.97;  
    if (duty<0.01)duty=0.01;
    duty=(float)(DUTYMAX-DUTYMIN)*duty+DUTYMIN;
    if (duty>DUTYMAX)duty=DUTYMAX;
    if (duty<DUTYMIN)duty=DUTYMIN;
    // pwm_set_chan_level(Slice_num_middle, PWM_CHAN_A, duty);  // GP20
    pwm_set_chan_level(Slice_num_rear, PWM_CHAN_B, duty);  // GP20
    //printf("%4.0f ", duty);
}

void set_duty_ml(float duty)
{
    if (duty>0.97)duty=0.97;  
    if (duty<0.01)duty=0.01;
    duty=(float)(DUTYMAX-DUTYMIN)*duty+DUTYMIN;
    if (duty>DUTYMAX)duty=DUTYMAX;
    if (duty<DUTYMIN)duty=DUTYMIN;
    // pwm_set_chan_level(Slice_num_rear, PWM_CHAN_B, duty);  // GP3
    pwm_set_chan_level(Slice_num_middle, PWM_CHAN_A, duty);  // GP3
    //printf("%4.0f ", duty);
}

void set_duty_mr(float duty)
{
    if (duty>0.97)duty=0.97;  
    if (duty<0.01)duty=0.01;
    duty=(float)(DUTYMAX-DUTYMIN)*duty+DUTYMIN;
    if (duty>DUTYMAX)duty=DUTYMAX;
    if (duty<DUTYMIN)duty=DUTYMIN;
    // pwm_set_chan_level(Slice_num_rear, PWM_CHAN_A, duty);  // GP2
    pwm_set_chan_level(Slice_num_middle, PWM_CHAN_B, duty);  // GP2
    //printf("%4.0f ", duty);
}


void payload_hook(void)
{
    pwm_set_chan_level(Slice_num_servo, PWM_CHAN_A, SERVO_HOOK);
    // printf("hook\n");
}

void payload_relese(void)
{
    pwm_set_chan_level(Slice_num_servo, PWM_CHAN_A, SERVO_RELEASE);
    // printf("release\n");
}