//control.cpp
//
//pull request


#include "control.hpp"
#include "nmpc.hpp"

//Sensor data
float Ax,Ay,Az,Wp,Wq,Wr,Mx,My,Mz,Mx0,My0,Mz0,Mx_ave,My_ave,Mz_ave;
float Acc_norm=0.0;

//Times
float Elapsed_time=0.0;
uint32_t S_time=0,E_time=0,D_time=0,S_time2=0,E_time2=0,D_time2=0;

//Counter
uint8_t AngleControlCounter=0;
uint16_t RateControlCounter=0;
uint16_t BiasCounter=0;
uint16_t LedBlinkCounter=0;

//Control 
float FR_duty, FL_duty, RR_duty, RL_duty;
float T_ref;
float Pbias=0.0,Qbias=0.0,Rbias=0.0;
float Phi_bias=0.0,Theta_bias=0.0,Psi_bias=0.0;
float Phi,Theta,Psi;
float Phi_ref=0.0,Theta_ref=0.0,Psi_ref=0.0;
float Elevator_center=0.0, Aileron_center=0.0, Rudder_center=0.0;
const float Phi_trim   = 0.01;
const float Theta_trim = 0.02;
const float Psi_trim   = 0.0;

// Core0/Core1 共有: NMPC が解いた最適 duty を ZOH 適用するためのバッファ
volatile float U_held[4] = {0.0f, 0.0f, 0.0f, 0.0f};

// NMPC 状態 (このファイル内に閉じる)
static nmpc::NmpcContext nmpc_ctx;
static float Nmpc_cost = 0.0f;
static uint8_t Nmpc_reset_flag = 0;

//Extended Kalman filter 
Matrix<float, 7 ,1> Xp = MatrixXf::Zero(7,1);
Matrix<float, 7 ,1> Xe = MatrixXf::Zero(7,1);
Matrix<float, 6 ,1> Z = MatrixXf::Zero(6,1);
Matrix<float, 3, 1> Omega_m = MatrixXf::Zero(3, 1);
Matrix<float, 3, 1> Oomega;
Matrix<float, 7, 7> P;
Matrix<float, 6, 6> Q;// = MatrixXf::Identity(3, 3)*0.1;
Matrix<float, 6, 6> R;// = MatrixXf::Identity(6, 6)*0.0001;
Matrix<float, 7 ,6> G;
Matrix<float, 3 ,1> Beta;

//Log
uint16_t LogdataCounter=0;
uint8_t Logflag=0;
volatile uint8_t Logoutputflag=0;
float Log_time=0.0;
const uint8_t DATANUM=38; //Log Data Number
const uint32_t LOGDATANUM=48000;
float Logdata[LOGDATANUM]={0.0};

//State Machine
uint8_t LockMode=0;
float Disable_duty =0.10;
float Flight_duty  =0.18;//0.2/////////////////
uint8_t OverG_flag = 0;

//Filter object
Filter acc_filter;

void loop_400Hz(void);
void rate_control(void);
void sensor_read(void);
void angle_control(void);
void output_data(void);
void output_sensor_raw_data(void);
void kalman_filter(void);
void logging(void);
void motor_stop(void);
uint8_t lock_com(void);
uint8_t logdata_out_com(void);
void printPQR(void);

#define AVERAGE 2000
#define KALMANWAIT 6000

//Main loop
//This function is called from PWM Intrupt on 400Hz.
void loop_400Hz(void)
{
  static uint8_t led=1;
  S_time=time_us_32();
  
  //割り込みフラグリセット
  pwm_clear_irq(2);


  if (Arm_flag==0)
  {
      //motor_stop();
      Elevator_center = 0.0;
      Aileron_center = 0.0;
      Rudder_center = 0.0;
      Pbias = 0.0;
      Qbias = 0.0;
      Rbias = 0.0;
      Phi_bias = 0.0;
      Theta_bias = 0.0;
      Psi_bias = 0.0;
      return;
  }
  else if (Arm_flag==1)
  {
    motor_stop();
    //Gyro Bias Estimate
    if (BiasCounter < AVERAGE)
    {
      //Sensor Read
      sensor_read();
      Aileron_center  += Chdata[3];
      Elevator_center += Chdata[1];
      Rudder_center   += Chdata[0];
      Pbias += Wp;
      Qbias += Wq;
      Rbias += Wr;
      Mx_ave += Mx;
      My_ave += My;
      Mz_ave += Mz;
      BiasCounter++;
      return;
    }
    else if(BiasCounter<KALMANWAIT)
    {
      //Sensor Read
      sensor_read();
      if(BiasCounter == AVERAGE)
      {
        Elevator_center = Elevator_center/AVERAGE;
        Aileron_center  = Aileron_center/AVERAGE;
        Rudder_center   = Rudder_center/AVERAGE;
        Pbias = Pbias/AVERAGE;
        Qbias = Qbias/AVERAGE;
        Rbias = Rbias/AVERAGE;
        Mx_ave = Mx_ave/AVERAGE;
        My_ave = My_ave/AVERAGE;
        Mz_ave = Mz_ave/AVERAGE;

        Xe(4,0) = Pbias;
        Xe(5,0) = Qbias;
        Xe(6,0) = Rbias;
        Xp(4,0) = Pbias;
        Xp(5,0) = Qbias;
        Xp(6,0) = Rbias;
        MN = Mx_ave;
        ME = My_ave;
        MD = Mz_ave;
      }
      
      AngleControlCounter++;
      if(AngleControlCounter==4)
      {
        AngleControlCounter=0;
        sem_release(&sem);
      
      }
      Phi_bias   += Phi;
      Theta_bias += Theta;
      Psi_bias   += Psi;
      BiasCounter++;
      return;
    }
    else
    {
      Arm_flag = 3;
      Phi_bias   = Phi_bias/KALMANWAIT;
      Theta_bias = Theta_bias/KALMANWAIT;
      Psi_bias   = Psi_bias/KALMANWAIT;
      return;
    }
  }
  else if( Arm_flag==2)
  {
    if(LockMode==2)
    {
      if(lock_com()==1)
      {
        LockMode=3;//Disenable Flight
        led=0;
        gpio_put(LED_PIN,led);
        return;
      }
      //Goto Flight
    }
    else if(LockMode==3)
    {
      if(lock_com()==0){
        LockMode=0;
        Arm_flag=3;
      }
      return;
    }
    //LED Blink
    gpio_put(LED_PIN, led);
    if(Logflag==1&&LedBlinkCounter<100){
      LedBlinkCounter++;
    }
    else
    {
      LedBlinkCounter=0;
      led=!led;
    }
   
    //Rate Control (400Hz)
    rate_control();
   
    if(AngleControlCounter==4)
    {
      AngleControlCounter=0;
      //Angle Control (100Hz)
      sem_release(&sem);
    }
    AngleControlCounter++;
  }
  else if(Arm_flag==3)
  {
    motor_stop();
    OverG_flag = 0;
    if(LedBlinkCounter<10){
      gpio_put(LED_PIN, 1);
      LedBlinkCounter++;
    }
    else if(LedBlinkCounter<100)
    {
      gpio_put(LED_PIN, 0);
      LedBlinkCounter++;
    }
    else LedBlinkCounter=0;
    
    //Get Stick Center 
    Aileron_center  = Chdata[3];
    Elevator_center = Chdata[1];
    Rudder_center   = Chdata[0];
  
    if(LockMode==0)
    {
      if( lock_com()==1)
      {
        LockMode=1;
        return;
      }
      //Wait  output log
    }
    else if(LockMode==1)
    {
      if(lock_com()==0)
      {
        LockMode=2;//Enable Flight
        Arm_flag=2;
      }
      return;
    }

    if(logdata_out_com()==1)
    {
      Arm_flag=4;
      return;
    }
  }
  else if(Arm_flag==4)
  {
    motor_stop();
    Logoutputflag=1;
    //LED Blink
    gpio_put(LED_PIN, led);
    if(LedBlinkCounter<400){
      LedBlinkCounter++;
    }
    else
    {
      LedBlinkCounter=0;
      led=!led;
    }
  }
  E_time=time_us_32();
  D_time=E_time-S_time;
}

void control_init(void)
{
  acc_filter.set_parameter(0.005, 0.0025);

  // NMPC 初期化 (フル再構築は最初の nmpc_step 呼び出し時に行われる)
  nmpc::nmpc_init(nmpc_ctx);

  for (int j = 0; j < 4; ++j) U_held[j] = 0.0f;
}

uint8_t lock_com(void)
{
  static uint8_t chatta=0,state=0;
  if( Chdata[2]<CH3MIN+80 
   && Chdata[0]>CH1MAX-80
   && Chdata[3]<CH4MIN+80 
   && Chdata[1]>CH2MAX-80)
  { 
    chatta++;
    if(chatta>50){
      chatta=50;
      state=1;
    }
  }
  else 
  {
    chatta=0;
    state=0;
  }

  return state;

}

uint8_t logdata_out_com(void)
{
  static uint8_t chatta=0,state=0;
  if( Chdata[4]<(CH5MAX+CH5MIN)*0.5 
   && Chdata[2]<CH3MIN+80 
   && Chdata[0]<CH1MIN+80
   && Chdata[3]>CH4MAX-80 
   && Chdata[1]>CH2MAX-80)
  {
    chatta++;
    if(chatta>50){
      chatta=50;
      state=1;
    }
  }
  else 
  {
    chatta=0;
    state=0;
  }

  return state;
}

void motor_stop(void)
{
  set_duty_fr(0.0);
  set_duty_fl(0.0);
  set_duty_rr(0.0);
  set_duty_rl(0.0);
}

// Core0 / 400Hz: センサ読みとり + 操縦桿スロットル取得 + 共有 U_held を ZOH 適用
//
// PID ミキシングは撤去。Core1 の NMPC が U_held[0..3] (FR, FL, RR, RL の duty) を
// 100Hz で更新し、本関数はそれを毎周期そのままモータへ反映する。
// 低スロットル時とオーバー G 検出時はモータ停止 (セーフティ既存ロジック保持)。
void rate_control(void)
{
  // Read Sensor Value (Wp, Wq, Wr, Ax/Ay/Az, Mx/My/Mz, Acc_norm 更新)
  sensor_read();

  // Throttle from stick (single-axis total thrust command)
  T_ref = 0.6 * BATTERY_VOLTAGE * (float)(Chdata[2] - CH3MIN) / (CH3MAX - CH3MIN);

  const float maximum_duty = 0.95f;

  // 低スロットル: モータ停止 & 操縦桿センター/姿勢バイアス更新
  if (T_ref / BATTERY_VOLTAGE < Disable_duty)
  {
    motor_stop();
    Aileron_center  = Chdata[3];
    Elevator_center = Chdata[1];
    Rudder_center   = Chdata[0];
    Phi_bias   = Phi;
    Theta_bias = Theta;
    Psi_bias   = Psi;
    return;
  }

  // オーバー G 検出時: モータ停止
  if (OverG_flag != 0)
  {
    motor_stop();
    return;
  }

  // ZOH: U_held を duty として印加 (Disable_duty〜maximum_duty へクランプ)
  float fr = U_held[0];
  float fl = U_held[1];
  float rr = U_held[2];
  float rl = U_held[3];

  if (fr < Disable_duty) fr = Disable_duty;
  if (fl < Disable_duty) fl = Disable_duty;
  if (rr < Disable_duty) rr = Disable_duty;
  if (rl < Disable_duty) rl = Disable_duty;
  if (fr > maximum_duty) fr = maximum_duty;
  if (fl > maximum_duty) fl = maximum_duty;
  if (rr > maximum_duty) rr = maximum_duty;
  if (rl > maximum_duty) rl = maximum_duty;

  set_duty_fr(fr);
  set_duty_fl(fl);
  set_duty_rr(rr);
  set_duty_rl(rl);

  FR_duty = fr;
  FL_duty = fl;
  RR_duty = rr;
  RL_duty = rl;
}

// Core1 / 100Hz: EKF 補正 → NMPC で 4 モータ duty を直接最適化
//
// 待機時 (T_ref/Vbat < Flight_duty) は NMPC をリセットし、U_held を最低 duty へ
// 固定する。実飛行中は nmpc_step() の出力 U_opt を U_held に書き込み、
// Core0 が ZOH で印加する。
void angle_control(void)
{
  while(1)
  {
    sem_acquire_blocking(&sem);
    sem_reset(&sem, 0);
    S_time2 = time_us_32();

    kalman_filter();

    const float q0 = Xe(0,0);
    const float q1 = Xe(1,0);
    const float q2 = Xe(2,0);
    const float q3 = Xe(3,0);
    const float e11 = q0*q0 + q1*q1 - q2*q2 - q3*q3;
    const float e12 = 2*(q1*q2 + q0*q3);
    const float e13 = 2*(q1*q3 - q0*q2);
    const float e23 = 2*(q2*q3 + q0*q1);
    const float e33 = q0*q0 - q1*q1 - q2*q2 + q3*q3;
    Phi   = atan2f(e23, e33);
    Theta = atan2f(-e13, sqrtf(e23*e23 + e33*e33));
    Psi   = atan2f(e12, e11);

    // 操縦桿から参照値
    Phi_ref   = Phi_trim   + 0.3f * (float)M_PI * (float)(Chdata[3] - (CH4MAX+CH4MIN)*0.5f) * 2.0f / (CH4MAX-CH4MIN);
    Theta_ref = Theta_trim + 0.3f * (float)M_PI * (float)(Chdata[1] - (CH2MAX+CH2MIN)*0.5f) * 2.0f / (CH2MAX-CH2MIN);
    Psi_ref   = Psi_trim   + 0.8f * (float)M_PI * (float)(Chdata[0] - (CH1MAX+CH1MIN)*0.5f) * 2.0f / (CH1MAX-CH1MIN);

    if (T_ref / BATTERY_VOLTAGE < Flight_duty)
    {
      // 待機モード: NMPC をリセットし U_held を Disable_duty へ
      nmpc::nmpc_init(nmpc_ctx);
      for (int j = 0; j < 4; ++j) U_held[j] = Disable_duty;
      Aileron_center  = Chdata[3];
      Elevator_center = Chdata[1];
      Rudder_center   = Chdata[0];
      Phi_bias   = Phi;
      Theta_bias = Theta;
      Psi_bias   = Psi;
      Nmpc_cost = 0.0f;
      Nmpc_reset_flag = 1;
    }
    else
    {
      // x0 = [phi-bias, theta-bias, p-bias, q-bias, r-bias]
      nmpc::StateVec x0;
      x0(0) = Phi   - Phi_bias;
      x0(1) = Theta - Theta_bias;
      x0(2) = Wp - Pbias;
      x0(3) = Wq - Qbias;
      x0(4) = Wr - Rbias;

      // 参照軌跡: 全ステージ共通の定常 ref
      //   x_ref = [Phi_ref, Theta_ref, 0, 0, Psi_ref]   (ヨーは角速度で表現)
      //   u_ref = ホバー相当 duty を 4 モータ均等に
      // 大きいバッファ (X_ref:400B, U_ref:320B) は static にしてスタック節約
      static nmpc::XStackVec X_ref;
      static nmpc::UStackVec U_ref;
      const float u_throttle = T_ref / BATTERY_VOLTAGE;
      for (int k = 0; k < nmpc::mpc::N; ++k) {
        const int xb = k * nmpc::mpc::NX;
        const int ub = k * nmpc::mpc::NU;
        X_ref(xb + 0) = Phi_ref;
        X_ref(xb + 1) = Theta_ref;
        X_ref(xb + 2) = 0.0f;
        X_ref(xb + 3) = 0.0f;
        X_ref(xb + 4) = Psi_ref;
        for (int j = 0; j < nmpc::mpc::NU; ++j) {
          U_ref(ub + j) = u_throttle;
        }
      }

      nmpc::InputVec U_opt;
      float cost = 0.0f;
      nmpc::nmpc_step(nmpc_ctx, x0, X_ref, U_ref, U_opt, cost);

      U_held[0] = U_opt(0);
      U_held[1] = U_opt(1);
      U_held[2] = U_opt(2);
      U_held[3] = U_opt(3);
      Nmpc_cost = cost;
      Nmpc_reset_flag = nmpc_ctx.last_was_reset ? 1 : 0;
    }

    logging();

    E_time2 = time_us_32();
    D_time2 = E_time2 - S_time2;
  }
}

// PID 撤去に伴い、旧 17-19 (Pref/Qref/Rref), 26-28 (P_com/Q_com/R_com),
// 29-33 (PID 内部状態) のスロットを NMPC 出力に再利用 (DATANUM=38 を維持)。
//   17-19: U_opt[0..2]                  (旧 Pref/Qref/Rref)
//   26   : U_opt[3]                     (旧 P_com)
//   27   : Nmpc_cost                    (旧 Q_com)
//   28   : Nmpc_reset_flag (0 or 1)     (旧 R_com)
//   29   : D_time2 [us, float 化]       (旧 p_pid.m_integral)
//   30   : last_admm_iters              (旧 q_pid.m_integral)
//   31   : cycles_since_reset           (旧 r_pid.m_integral)
//   32-33: 予約 (0)                     (旧 phi_pid/theta_pid integral)
void logging(void)
{
  if(Chdata[4]>(CH5MAX+CH5MIN)*0.5)
  {
    if(Logflag==0)
    {
      Logflag=1;
      LogdataCounter=0;
    }
    if(LogdataCounter+DATANUM<LOGDATANUM)
    {
      Logdata[LogdataCounter++]=Xe(0,0);                  //1
      Logdata[LogdataCounter++]=Xe(1,0);                  //2
      Logdata[LogdataCounter++]=Xe(2,0);                  //3
      Logdata[LogdataCounter++]=Xe(3,0);                  //4
      Logdata[LogdataCounter++]=Xe(4,0);                  //5
      Logdata[LogdataCounter++]=Xe(5,0);                  //6
      Logdata[LogdataCounter++]=Xe(6,0);                  //7
      Logdata[LogdataCounter++]=Wp;                       //8
      Logdata[LogdataCounter++]=Wq;                       //9
      Logdata[LogdataCounter++]=Wr;                       //10

      Logdata[LogdataCounter++]=Ax;                       //11
      Logdata[LogdataCounter++]=Ay;                       //12
      Logdata[LogdataCounter++]=Az;                       //13
      Logdata[LogdataCounter++]=Mx;                       //14
      Logdata[LogdataCounter++]=My;                       //15
      Logdata[LogdataCounter++]=Mz;                       //16
      Logdata[LogdataCounter++]=U_held[0];                //17  NMPC U_opt[0]
      Logdata[LogdataCounter++]=U_held[1];                //18  NMPC U_opt[1]
      Logdata[LogdataCounter++]=U_held[2];                //19  NMPC U_opt[2]
      Logdata[LogdataCounter++]=Phi-Phi_bias;             //20

      Logdata[LogdataCounter++]=Theta-Theta_bias;         //21
      Logdata[LogdataCounter++]=Psi-Psi_bias;             //22
      Logdata[LogdataCounter++]=Phi_ref;                  //23
      Logdata[LogdataCounter++]=Theta_ref;                //24
      Logdata[LogdataCounter++]=Psi_ref;                  //25
      Logdata[LogdataCounter++]=U_held[3];                //26  NMPC U_opt[3]
      Logdata[LogdataCounter++]=Nmpc_cost;                //27  NMPC コスト
      Logdata[LogdataCounter++]=(float)Nmpc_reset_flag;   //28  NMPC リセットフラグ
      Logdata[LogdataCounter++]=(float)D_time2;           //29  NMPC 計算時間 [us]
      Logdata[LogdataCounter++]=(float)nmpc_ctx.last_admm_iters;     //30

      Logdata[LogdataCounter++]=(float)nmpc_ctx.cycles_since_reset;  //31
      Logdata[LogdataCounter++]=0.0f;                     //32  予約
      Logdata[LogdataCounter++]=0.0f;                     //33  予約
      Logdata[LogdataCounter++]=Pbias;                    //34
      Logdata[LogdataCounter++]=Qbias;                    //35

      Logdata[LogdataCounter++]=Rbias;                    //36
      Logdata[LogdataCounter++]=T_ref;                    //37
      Logdata[LogdataCounter++]=Acc_norm;                 //38
    }
    else Logflag=2;
  }
  else
  {
    if(Logflag>0)
    {
      Logflag=0;
      LogdataCounter=0;
    }
  }
}

void log_output(void)
{
  if(LogdataCounter==0)
  {
    printPQR();
    printf("#NMPC: N=%d dt=%.4f NX=%d NU=%d reset_period=%d admm_iter=%d\n",
           nmpc::mpc::N, (double)nmpc::mpc::dt, nmpc::mpc::NX, nmpc::mpc::NU,
           nmpc::rti::reset_period, nmpc::rti::admm_max_iter);
  }
  if(LogdataCounter+DATANUM<LOGDATANUM)
  {
    //LockMode=0;
    printf("%10.2f ", Log_time);
    Log_time=Log_time + 0.01;
    for (uint8_t i=0;i<DATANUM;i++)
    {
      printf("%12.5f",Logdata[LogdataCounter+i]);
    }
    printf("\n");
    LogdataCounter=LogdataCounter + DATANUM;
  }
  else 
  {
    Arm_flag=3;
    Logoutputflag=0;
    LockMode=0;
    Log_time=0.0;
    LogdataCounter=0;
  }
}


void gyroCalibration(void)
{
  float wp,wq,wr;
  float sump,sumq,sumr;
  uint16_t N=400;
  for(uint16_t i=0;i<N;i++)
  {
    sensor_read();
    sump=sump+Wp;
    sumq=sumq+Wq;
    sumr=sumr+Wr;
  }
  Pbias=sump/N;
  Qbias=sumq/N;
  Rbias=sumr/N;
}

void sensor_read(void)
{
  float mx1, my1, mz1, mag_norm, acc_norm, rate_norm;

  imu_mag_data_read();
  Ax =-acceleration_mg[0]*GRAV*0.001;
  Ay =-acceleration_mg[1]*GRAV*0.001;
  Az = acceleration_mg[2]*GRAV*0.001;
  Wp = angular_rate_mdps[0]*M_PI*5.55555555e-6;//5.5.....e-6=1/180/1000
  Wq = angular_rate_mdps[1]*M_PI*5.55555555e-6;
  Wr =-angular_rate_mdps[2]*M_PI*5.55555555e-6;
  Mx0 =-magnetic_field_mgauss[0];
  My0 = magnetic_field_mgauss[1];
  Mz0 =-magnetic_field_mgauss[2];

  
  acc_norm = sqrt(Ax*Ax + Ay*Ay + Az*Az);
  if (acc_norm>250.0) OverG_flag = 1;
  Acc_norm = acc_filter.update(acc_norm);
  rate_norm = sqrt(Wp*Wp + Wq*Wq + Wr*Wr);
  if (rate_norm > 6.0) OverG_flag =1;

/*地磁気校正データ
回転行列
[[ 0.65330968  0.75327755 -0.07589064]
 [-0.75666134  0.65302622 -0.03194321]
 [ 0.02549647  0.07829232  0.99660436]]
中心座標
122.37559195017053 149.0184454603531 -138.99116060635413
W
-2.432054387460946
拡大係数
0.003077277151877191 0.0031893151610213463 0.0033832794976645804

//回転行列
const float rot[9]={0.65330968, 0.75327755, -0.07589064,
                   -0.75666134, 0.65302622, -0.03194321,
                    0.02549647, 0.07829232,  0.99660436};
//中心座標
const float center[3]={122.37559195017053, 149.0184454603531, -138.99116060635413};
//拡大係数
const float zoom[3]={0.003077277151877191, 0.0031893151610213463, 0.0033832794976645804};
*/
  //回転行列
  const float rot[9]={-0.78435472, -0.62015392, -0.01402787,
                       0.61753358, -0.78277935,  0.07686857,
                      -0.05865107,  0.05162955,  0.99694255};
  //中心座標
  const float center[3]={-109.32529343620176, 72.76584808916506, 759.2285249891385};
  //拡大係数
  const float zoom[3]={0.002034773458122364, 0.002173892202021849, 0.0021819494099235273};

//回転・平行移動・拡大
  mx1 = zoom[0]*( rot[0]*Mx0 +rot[1]*My0 +rot[2]*Mz0 -center[0]);
  my1 = zoom[1]*( rot[3]*Mx0 +rot[4]*My0 +rot[5]*Mz0 -center[1]);
  mz1 = zoom[2]*( rot[6]*Mx0 +rot[7]*My0 +rot[8]*Mz0 -center[2]);
//逆回転
  Mx = rot[0]*mx1 +rot[3]*my1 +rot[6]*mz1;
  My = rot[1]*mx1 +rot[4]*my1 +rot[7]*mz1;
  Mz = rot[2]*mx1 +rot[5]*my1 +rot[8]*mz1; 

  mag_norm=sqrt(Mx*Mx +My*My +Mz*Mz);
  Mx/=mag_norm;
  My/=mag_norm;
  Mz/=mag_norm;
}

void variable_init(void)
{
  //Variable Initalize
  Xe << 1.00, 0.0, 0.0, 0.0,0.0,0.0, 0.0;
  Xp =Xe;

  Q <<  6.0e-5, 0.0    , 0.0    ,  0.0    , 0.0    , 0.0   ,
        0.0   , 5.0e-5 , 0.0    ,  0.0    , 0.0    , 0.0   ,
        0.0   , 0.0    , 2.8e-5 ,  0.0    , 0.0    , 0.0   ,
        0.0   , 0.0    , 0.0    ,  5.0e-5 , 0.0    , 0.0   ,
        0.0   , 0.0    , 0.0    ,  0.0    , 5.0e-5 , 0.0   ,
        0.0   , 0.0    , 0.0    ,  0.0    , 0.0    , 5.0e-5;

  R <<  1.701e0, 0.0     , 0.0     , 0.0   , 0.0   , 0.0   ,
        0.0     , 2.799e0, 0.0     , 0.0   , 0.0   , 0.0   ,
        0.0     , 0.0     , 1.056e0, 0.0   , 0.0   , 0.0   ,
        0.0     , 0.0     , 0.0     , 2.3e-1, 0.0   , 0.0   ,
        0.0     , 0.0     , 0.0     , 0.0   , 1.4e-1, 0.0   ,
        0.0     , 0.0     , 0.0     , 0.0   , 0.0   , 0.49e-1;
          
  G <<   1.0, 1.0, 1.0, 0.0, 0.0, 0.0, 
        -1.0, 1.0,-1.0, 0.0, 0.0, 0.0, 
        -1.0,-1.0, 1.0, 0.0, 0.0, 0.0, 
         1.0,-1.0,-1.0, 0.0, 0.0, 0.0, 
         0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 
         0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 
         0.0, 0.0, 0.0, 0.0, 0.0, 1.0;
  
  G=G*0.01;

  Beta << 0.0, 0.0, 0.0;
  
  P <<  1e0,  0,   0,   0,   0,  0,   0,  
        0  ,1e0,   0,   0,   0,  0,   0,
        0  ,  0, 1e0,   0,   0,  0,   0,  
        0  ,  0,   0, 1e0,   0,  0,   0, 
        0  ,  0,   0, 0  , 1e0,  0,   0,  
        0  ,  0,   0, 0  ,   0,1e0,   0,  
        0  ,  0,   0, 0  ,   0,  0, 1e0;
}

void printPQR(void)
{
  volatile int m=0;
  volatile int n=0;
  //Print P
  printf("#P\n");
  for (m=0;m<7;m++)
  {
    printf("# ");
    for (n=0;n<7;n++)
    {
      printf("%12.4e ",P(m,n));
    }
    printf("\n");
  }
  //Print Q
  printf("#Q\n");
  for (m=0;m<6;m++)
  {
    printf("# ");
    for (n=0;n<6;n++)
    {
      printf("%12.4e ",Q(m,n));
    }
    printf("\n");
  }
  //Print R
  printf("#R\n");
  for (m=0;m<6;m++)
  {
    printf("# ");
    for (n=0;n<6;n++)
    {
      printf("%12.4e ",R(m,n));
    }
    printf("\n");
  }
}

void output_data(void)
{
  printf("%9.3f,"
         "%13.8f,%13.8f,%13.8f,%13.8f,"
         "%13.8f,%13.8f,%13.8f,"
         "%6lu,%6lu,"
         "%13.8f,%13.8f,%13.8f,"
         "%13.8f,%13.8f,%13.8f,"
         "%13.8f,%13.8f,%13.8f"
         //"%13.8f"
         "\n"
            ,Elapsed_time//1
            ,Xe(0,0), Xe(1,0), Xe(2,0), Xe(3,0)//2~5 
            ,Xe(4,0), Xe(5,0), Xe(6,0)//6~8
            //,Phi-Phi_bias, Theta-Theta_bias, Psi-Psi_bias//6~8
            ,D_time, D_time2//10,11
            ,Ax, Ay, Az//11~13
            ,Wp, Wq, Wr//14~16
            ,Mx, My, Mz//17~19
            //,mag_norm
        ); //20
}
void output_sensor_raw_data(void)
{
  printf("%9.3f,"
         "%13.5f,%13.5f,%13.5f,"
         "%13.5f,%13.5f,%13.5f,"
         "%13.5f,%13.5f,%13.5f"
         "\n"
            ,Elapsed_time//1
            ,Ax, Ay, Az//2~4
            ,Wp, Wq, Wr//5~7
            ,Mx, My, Mz//8~10
        ); //20
}

void kalman_filter(void)
{
  //Kalman Filter
  float dt=0.01;
  Omega_m << Wp, Wq, Wr;
  Z << Ax, Ay, Az, Mx, My, Mz;
  ekf(Xp, Xe, P, Z, Omega_m, Q, R, G*dt, Beta, dt);
}


Filter::Filter()
{
  m_state = 0.0;
  m_T = 0.0025;
  m_h = 0.0025;
}

void Filter::reset(void)
{
  m_state = 0.0;
}

void Filter::set_parameter(float T, float h)
{
  m_T = T;
  m_h = h;
}

float Filter::update(float u)
{
  m_state = m_state * m_T /(m_T + m_h) + u * m_h/(m_T + m_h);
  m_out = m_state;
  return m_out;
}
