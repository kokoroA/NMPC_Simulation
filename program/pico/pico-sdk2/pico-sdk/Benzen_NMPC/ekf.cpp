/**
 Copyright (c) 2021 Kouhei Ito 
*/


#include "ekf.hpp"

float MN,ME,MD;


//Initilize White Noise
std::random_device rnd;
std::mt19937 mt(rnd());  
std::normal_distribution<> norm(0.0, 1.0);

//Kalman Horizontal 
Matrix<float, 2 ,2> Sigma_Yn_est_v = MatrixXf::Zero(3,3);
Matrix<float, 2,2> Sigma_Yn_pre_v = MatrixXf::Zero(3,3);
Matrix<float, 1,2> observation_mat_v = MatrixXf::Zero(1,3);
Matrix<float, 2,1> observation_mat_transposed_v = MatrixXf::Zero(3,1);
Matrix<float, 2,1> control_mat_v = MatrixXf::Zero(3,1);
Matrix<float, 2,2> unit_mat_v = MatrixXf::Zero(3,3);
Matrix<float, 2,2> system_mat_v = MatrixXf::Zero(3,3);
Matrix<float, 1, 1> Kal_element_v;
Matrix<float, 3, 2> K_v;
Matrix<float, 1, 1> R_mat_v = MatrixXf::Zero(2,2);
// Matrix<float, 2, 1> R = MatrixXf::Zero(2,1);
Matrix<float,1,1> Kal_element_inv_v = Kal_element.inverse();
Matrix<float, 2,2> last_Sigma_Yn_pre_v = MatrixXf::Zero(3,3);
Matrix<float, 2,1> mu_Yn_pre_v  = MatrixXf::Zero(3,1);
Matrix<float, 2,1> last_mu_Yn_pre_v = MatrixXf::Zero(3,1);
Matrix<float, 2,1> mu_Yn_est_v = MatrixXf::Zero(3,1);
Matrix<float, 2,2> Q_mat_v = MatrixXf::Zero(3,3);
Matrix<float, 2,2> yn_mat_v;
Matrix<float, 2,2> k_inv_v;

//Kalman Altitude
Matrix<float, 2 ,2> Sigma_Yn_est = MatrixXf::Zero(2,2);
Matrix<float, 2,2> Sigma_Yn_pre = MatrixXf::Zero(2,2);
Matrix<float, 1,2> observation_mat = MatrixXf::Zero(1,2);
Matrix<float, 2,1> observation_mat_transposed = MatrixXf::Zero(2,1);
Matrix<float, 2,1> control_mat = MatrixXf::Zero(2,1);
Matrix<float, 2,2> unit_mat = MatrixXf::Zero(2,2);
Matrix<float, 2,2> system_mat = MatrixXf::Zero(2,2);
Matrix<float, 1, 1> Kal_element;
Matrix<float, 2, 1> K;
Matrix<float, 1, 1> R_mat = MatrixXf::Zero(1,1);
// Matrix<float, 2, 1> R = MatrixXf::Zero(2,1);
Matrix<float,1,1> Kal_element_inv = Kal_element.inverse();
Matrix<float, 2,2> last_Sigma_Yn_pre = MatrixXf::Zero(2,2);
Matrix<float, 2,1> mu_Yn_pre  = MatrixXf::Zero(2,1);
Matrix<float, 2,1> last_mu_Yn_pre = MatrixXf::Zero(2,1);
Matrix<float, 2,1> mu_Yn_est = MatrixXf::Zero(2,1);
Matrix<float, 2,2> Q_mat = MatrixXf::Zero(2,2);
Matrix<float, 1,1> u_n  = MatrixXf::Zero(1,1);
Matrix<float, 1,1> u_n_v  = MatrixXf::Zero(1,1);
Matrix<float, 1,1> yn_mat;
Matrix<float, 1,1> k_inv;
// float stdv_Q = 0.08;
float stdv_Q = 0.01;
float stdv_Q_v = 0.025;
// float stdv_Q = 0.5;
float Q_k = std::pow(stdv_Q,2.0);
float Q_k_v = std::pow(stdv_Q_v,2.0);
float error = 0;
float error_v = 0;
float r = 0;
float r_v = 0; 
float last_error = 0;
float last_error_v = 0;
float Control_T = 0.02;
float de = 0;
float ie = 0;
float de_v = 0;
float ie_v = 0;
float observe_y;

// float Kp = 0.044; // 0.08   0.042
float Kp = 0.06; //Benzen  T_ref:0.6:0.045  0.05
// float Ki = 900;//400Hz
//float Ki = 128;//57 58 Hz    129  128.571
//float Ki = 112.5;//50Hz
// float Ki = 31;//40Hz  90  45  43  42
float Ki = 100000; //Benzen 100000 100
float Kd = 0;
// float Kp_v = 0.0026; //0.003 0.00172
float Kp_v = 0.00025; //Benzen  T_ref:0.6:0.0002  0.00025
// float Ki_v = 300;//400Hz
//float Ki_v = 42;//57 58Hz    43 42.857
//float Ki_v = 37.5;//50Hz
// float Ki_v = 4;//40Hz  30  15  13  12
float Ki_v = 100000;//Benzen 100000 100
float Kd_v = 0;
float u = 0;
float u_v = 0;
// float h_kalman = 0.02;
float h_kalman = 0.025;
float m = 0.8;
float stdv_R = 0.035;
// float stdv_R = 7.965;
// float stdv_R = 0.7965;
float integral = 0;
float differential = 0;
float integral_v = 0;
float differential_v = 0;
float eta = 0.125;

//Kalmanfilter for Horizontal
float h_horizontal = 0.025;
// float h_horizontal = 0.2;
//推定,予測する状態
float Xn_pre_1 = 0;
float Xn_pre_2 = 0;
float Xn_pre_3 = 0;
float Xn_est_1 = 0;
float Xn_est_2 = 0;
float Xn_est_3 = 0;
//推定、予測する誤差の共分散
float p11_pre = 1;
float p12_pre = 0;
float p13_pre = 0; 
float p21_pre = 0;
float p22_pre = 1; 
float p23_pre = 0; 
float p31_pre = 0; 
float p32_pre = 0; 
float p33_pre = 1; 
float p11_est = 1;
float p12_est = 0;
float p13_est = 0; 
float p21_est = 0;
float p22_est = 1; 
float p23_est = 0; 
float p31_est = 0; 
float p32_est = 0; 
float p33_est = 1; 
//イノベーション
float e1 = 0;
float e2 = 0;
//カルマンゲイン計算のための変数
float s11 = 0;
float s12 = 0;
float s13 = 0;
float s21 = 0;
float s22 = 0;
float s23 = 0;
float s_i11 = 0;
float s_i12 = 0;
float s_i13 = 0;
float s_i21 = 0;
float s_i22 = 0;
float s_i23 = 0;
//カルマンゲイン
float K11 = 0;
float K12 = 0;
float K21 = 0;
float K22 = 0;
float K31 = 0;
float K32 = 0;
//定数
float g = 9.81;
float u0 = 0.994522;
float w0 = -0.104528;
float Cv = 0.311026;
float V0 = 0;
float delta_p = 0;
float delta_r = 0;
float theta0 = -0.10472;
float delta_phi = 0;
float delta_psi = 0;
//離散化した運動方程式の要素
float f11 = 1 - ((2 * Cv * V0 * h_horizontal) / m);
float f12 = 0;
float f13 = 0;
float f21 = h_horizontal;
float f22 = 1;
float f23 = ((u0 * cos(theta0)) + (w0 * sin(theta0))) * h_horizontal;
float f31 = 0;
float f32 = 0;
float f33 = 1;
//離散化した制御行列の要素
float u1 = ((-u0 * delta_r) + (w0 * delta_p) + (g * cos(theta0) * delta_phi)) * h_horizontal;
float u2 = -w0 * delta_phi * h_horizontal;
float u3 = (delta_r * h_horizontal)/cos(theta0);
//システムノイズ
float q1 = 0.01; //速度v
float q2 = 0.0015; //位置Y
float q3 = 0.01; //角度Ψ
//観測の共分散
float r1 = 0.000001;
float r2 = 0.001;


float initialize( Matrix<float, 2 ,2> &Sigma_Yn_est,
                    Matrix<float, 2,2> &Sigma_Yn_pre,
                    Matrix<float, 1,2> &observation_mat,
                    Matrix<float, 2,1> &observation_mat_transposed,
                    Matrix<float, 2,1> &control_mat,
                    Matrix<float, 2,2> &unit_mat,
                    Matrix<float, 2,2> &system_mat,
                    Matrix<float, 1, 1> &Kal_element,
                    Matrix<float, 2, 1> &K,
                    Matrix<float, 1, 1> &R_mat,
                    Matrix<float,1,1> &Kal_element_inv,
                    Matrix<float, 2,2> &Q_mat
                  )

{
    Sigma_Yn_est(0,0) = 1.0;
    Sigma_Yn_est(0,1) = 0.0;
    Sigma_Yn_est(1,0) = 0.0;
    Sigma_Yn_est(1,1) = 1.0;
    Sigma_Yn_pre(0,0) = 1;
    Sigma_Yn_pre(0,1) = 0;
    Sigma_Yn_pre(1,0) = 0;
    Sigma_Yn_pre(1,1) = 1;
    control_mat(0,0) = h_kalman/m;
    control_mat(1,0) = 0;
    unit_mat(0,0) = 1;
    unit_mat(0,1) = 0;
    unit_mat(1,0) = 0;
    unit_mat(1,1) = 1;
    system_mat(0,0) = 1;
    system_mat(0,1) = 0;
    system_mat(1,0) = h_kalman;
    system_mat(1,1) = 1;
    observation_mat(0,0) = 0.0;
    observation_mat(0,1) = 1.0;
    R_mat(0,0) = std::pow(stdv_R,2.0);
    // R = std::pow(stdv_R,2.0);
    Q_mat(0,0) = Q_k_v;
    Q_mat(0,1) = 0;
    Q_mat(1,0) = 0;
    Q_mat(1,1) = Q_k;
    return 0;
}

float Kalman_holizontal(float camera_y,float camera_psi,float deltaP,float deltaR,float deltaPhi){

  //離散化した運動方程式の要素
  f11 = 1-((2 * Cv * V0 * h_horizontal) / m);
  f23 = ((u0 * cos(theta0)) + (w0 * sin(theta0))) * h_horizontal;

  //離散化した制御行列の要素
  u1 = ((-u0 * deltaR )+ (w0 * deltaP) + (g * cos(theta0) * deltaPhi)) * h_horizontal;
  u2 = -w0 * deltaPhi * h_horizontal;
  u3 = (deltaR * h_horizontal)/cos(theta0);

  //Xの予測
  Xn_pre_1 = (f11 * Xn_est_1) + (f12 * Xn_est_2) +(f13 * Xn_est_3) + u1;
  Xn_pre_2 = (f21 * Xn_est_1) + (f22 * Xn_est_2) +(f23 * Xn_est_3) + u2;
  Xn_pre_3 = (f31 * Xn_est_1) + (f32 * Xn_est_2) +(f33 * Xn_est_3) + u3;

  //誤差共分散の予測
  p11_pre = (f11 * f11 * p11_est) + q1;
  p12_pre = (f11 * f21 * p11_est) + (f11 * p12_est) + (f11 * f23 * p13_est) ;
  p13_pre = f11 * p13_est;
  p21_pre = f11 * ((f21 * p11_est) + p21_est + (f23 * p31_est));
  p22_pre = f21 * ((f21 * p11_est) + p21_est + (f23 * p31_est)) + (f21 * p12_est) + p22_est + (f23 * p32_est) + (f23 * ((f21 * p13_est) + p23_est + (f23 * p33_est))) + q2; 
  p23_pre = (f21 * p13_est) + p23_est + (f23 * p33_est); 
  p31_pre = p31_est * f11; 
  p32_pre = (p31_est * f21) + p32_est + (p33_est * f23); 
  p33_pre = p33_est + q3; 

  //イノベーション
  e1 = camera_y - Xn_pre_2;
  e2 = camera_psi - Xn_pre_3;

  //カルマンゲインの計算
  s11 = r1 + p22_est;
  s12 = p23_pre;
  s21 = p32_pre;
  s22 = r2 + p33_pre;
  s_i11 = s22 / (s11*s22 - s12*s21);
  s_i12 = -s12 / (s11*s22 - s12*s21);
  s_i21 = -s21 / (s11*s22 - s12*s21);
  s_i22 = s11 / (s11*s22 - s12*s21);
  K11 = (p12_pre * s_i11) + (p13_pre * s_i21);
  K12 = (p12_pre * s_i12) + (p13_pre * s_i22);
  K21 = (p22_pre * s_i11) + (p23_pre * s_i21);
  K22 = (p22_pre * s_i12) + (p23_pre * s_i22);
  K31 = (p32_pre * s_i11) + (p33_pre * s_i21);
  K32 = (p32_pre * s_i12) + (p33_pre * s_i22);


  //状態の推定
  Xn_est_1 = Xn_pre_1 + (K11*e1 )+ (K12*e2); //速度
  Xn_est_2 = Xn_pre_2 + (K21*e1) + (K22*e2);  //横ズレ
  Xn_est_3 = Xn_pre_3 + (K31*e1) + (K32*e2);  //角度ズレ


  //誤差の共分散の推定
  p11_est = p11_pre - (K11*p21_pre) -(K12*p31_pre);
  p12_est = p12_pre - (K11*p22_pre) -(K12*p32_pre);
  p13_est = p13_pre - (K11*p23_pre) - (K12*p33_pre);

  p21_est = p21_pre * (1 - K21) - (K22*p31_pre);
  p22_est = p22_pre * (1 - K21) - (K22*p32_pre); 
  p23_est = p23_pre * (1 - K21) - (K22*p33_pre);

  p31_est = -(K31*p21_pre) + (p31_pre*(1 - K32));
  p32_est = -(K31*p22_pre) + (p32_pre*(1 - K32));
  p33_est = -(K31*p23_pre) + (p33_pre*(1 - K32));

  return 1;
}

float Kalman_PID(float observe_y,float Ax)
{
    yn_mat(0,0) = observe_y;
    //値の更新
    last_mu_Yn_pre = mu_Yn_est;
    last_Sigma_Yn_pre = Sigma_Yn_est;

    //カルマンフィルタ
    mu_Yn_pre = (system_mat * last_mu_Yn_pre) + (control_mat*(Ax));
    Sigma_Yn_pre = (system_mat * last_Sigma_Yn_pre * system_mat.transpose()) + Q_mat;
    k_inv = ((observation_mat * Sigma_Yn_pre * observation_mat.transpose()) + R_mat);
    K = (Sigma_Yn_pre * observation_mat.transpose()) / k_inv(0,0);
    mu_Yn_est = mu_Yn_pre + K * (observe_y - (observation_mat * mu_Yn_pre));
    Sigma_Yn_est = (unit_mat - (K * observation_mat)) * Sigma_Yn_pre;
    
    // return u_v;
    return mu_Yn_est(1,0);
    //return 1;
}

float alt_PID(float ref_alt){
    error =  ref_alt - mu_Yn_est(1,0) ;
    integral = integral + (h_kalman * (error + last_error)) / (2 * Ki);
    differential = (((2 * eta * Kd - h_kalman) * differential) / (2 * eta * Kd + h_kalman)) + ((2 * Kd) * (error - last_error)) / (2 * eta * Kd + h_kalman);
    u_n(0,0) = Kp * (error + integral + differential);
    u = u_n(0,0);

    last_error = error;

    // //PID for velocity
    error_v = u - mu_Yn_est(0,0);
    integral_v = integral_v + (h_kalman * (error_v + last_error_v)) / (2 * Ki_v);
    differential_v = (((2 * eta * Kd_v - h_kalman) * differential) / (2 * eta * Kd_v + h_kalman)) + ((2 * Kd_v) * (error_v - last_error_v)) / (2 * eta * Kd_v + h_kalman);
    u_n_v(0,0) = Kp_v * (error_v + integral_v + differential_v);
    u_v = u_n_v(0,0);

    last_error_v = error_v;


    return u_v;
}

void Kalman_init(void){
  initialize(
          Sigma_Yn_est,
          Sigma_Yn_pre,
          observation_mat,
          observation_mat_transposed,
          control_mat,
          unit_mat,
          system_mat,
          Kal_element,
          K,
          R_mat,
          Kal_element_inv,
          Q_mat);
}

//Runge-Kutta Method 
uint8_t rk4(uint8_t (*func)(float t, 
            Matrix<float, 7, 1> x, 
            Matrix<float, 3, 1> omega, 
            Matrix<float, 3, 1> beta, 
            Matrix<float, 7, 1> &k),
            float t, 
            float h, 
            Matrix<float, 7 ,1> &x, 
            Matrix<float, 3, 1> omega,
            Matrix<float, 3, 1> beta)
{
  Matrix<float, 7, 1> k1,k2,k3,k4;

  func(t,       x,            omega, beta, k1);
  func(t+0.5*h, x + 0.5*h*k1, omega, beta, k2);
  func(t+0.5*h, x + 0.5*h*k2, omega, beta, k3);
  func(t+h,     x +     h*k3, omega, beta, k4);
  x = x + h*(k1 + 2.0*k2 + 2.0*k3 + k4)/6.0;
  
  return 0;
}

//Continuous Time State Equation for simulation
uint8_t xdot( float t, 
              Matrix<float, 7, 1> x, 
              Matrix<float, 3, 1> omega , 
              Matrix<float, 3, 1> beta, 
              Matrix<float, 7, 1> &k)
{
  float q0 = x(0,0);
  float q1 = x(1,0);
  float q2 = x(2,0);
  float q3 = x(3,0);
  float dp = x(4,0);
  float dq = x(5,0);
  float dr = x(6,0);
  float p = omega(0,0);
  float q = omega(1,0);
  float r = omega(2,0);
  float betax = beta(0,0);
  float betay = beta(1,0);
  float betaz = beta(2,0);
 
  k(0,0) = 0.5*(-p*q1 - q*q2 - r*q3);
  k(1,0) = 0.5*( p*q0 + r*q2 - q*q3);
  k(2,0) = 0.5*( q*q0 - r*q1 + p*q3);
  k(3,0) = 0.5*( r*q0 + q*q1 - p*q2);
  k(4,0) =-betax*dp + norm(mt);
  k(5,0) =-betay*dq + norm(mt);
  k(6,0) =-betaz*dr + norm(mt);

  return 0;
}

//Discrite Time State Equation
uint8_t state_equation( Matrix<float, 7, 1> &xe, 
                        Matrix<float, 3, 1> omega_m, 
                        Matrix<float, 3, 1> beta, 
                        float dt,
                        Matrix<float, 7, 1> &xp)
{
  float q0=xe(0, 0);
  float q1=xe(1, 0);
  float q2=xe(2, 0);
  float q3=xe(3, 0);
  float dp=xe(4, 0);
  float dq=xe(5, 0);
  float dr=xe(6, 0);
  float pm=omega_m(0, 0);
  float qm=omega_m(1, 0);
  float rm=omega_m(2, 0);
  float betax=beta(0, 0);
  float betay=beta(1, 0);
  float betaz=beta(2, 0);

  xp(0, 0) = q0 + 0.5*(-(pm-dp)*q1 -(qm-dq)*q2 -(rm-dr)*q3)*dt;
  xp(1, 0) = q1 + 0.5*( (pm-dp)*q0 +(rm-dr)*q2 -(qm-dq)*q3)*dt;
  xp(2, 0) = q2 + 0.5*( (qm-dq)*q0 -(rm-dr)*q1 +(pm-dp)*q3)*dt;
  xp(3, 0) = q3 + 0.5*( (rm-dr)*q0 +(qm-dq)*q1 -(pm-dp)*q2)*dt;
  xp(4, 0) = dp -betax*dp*dt;
  xp(5, 0) = dq -betay*dq*dt;
  xp(6, 0) = dr -betaz*dr*dt;
  
  return 0;
}

//Observation Equation
uint8_t observation_equation( Matrix<float, 7, 1>x, 
                              Matrix<float, 6, 1>&z, 
                              float g, 
                              float mn, 
                              float me, 
                              float md)
{
  float q0 = x(0, 0);
  float q1 = x(1, 0);
  float q2 = x(2, 0);
  float q3 = x(3, 0);

  z(0, 0) = 2.0*(q1*q3 - q0*q2)*g;
  z(1, 0) = 2.0*(q2*q3 + q0*q1)*g;
  z(2, 0) = (q0*q0 - q1*q1 - q2*q2 + q3*q3)*g ;
  z(3, 0) = (q0*q0 + q1*q1 - q2*q2 - q3*q3)*mn + 2.0*(q1*q2 + q0*q3)*me + 2.0*(q1*q3 - q0*q2)*md;
  z(4, 0) = 2.0*(q1*q2 - q0*q3)*mn + (q0*q0 - q1*q1 + q2*q2 -q3*q3)*me + 2.0*(q2*q3 + q0*q1)*md;
  z(5, 0) = 2.0*(q1*q3 + q0*q2)*mn + 2.0*(q2*q3 - q0*q1)*me + (q0*q0 - q1*q1 - q2*q2 + q3*q3)*md;
  
  return 0;
}

//Make Jacobian matrix F
uint8_t F_jacobian( Matrix<float, 7, 7>&F, 
                    Matrix<float, 7, 1> x, 
                    Matrix<float, 3, 1> omega, 
                    Matrix<float, 3, 1> beta, 
                    float dt)
{
  float q0=x(0, 0);
  float q1=x(1, 0);
  float q2=x(2, 0);
  float q3=x(3, 0);
  float dp=x(4, 0);
  float dq=x(5, 0);
  float dr=x(6, 0);
  float pm=omega(0, 0);
  float qm=omega(1, 0);
  float rm=omega(2, 0);
  float betax=beta(0, 0);
  float betay=beta(1, 0);
  float betaz=beta(2, 0);

  //x(0, 0) = q0 + 0.5*(-(pm-dp)*q1 -(qm-dq)*q2 -(rm-dr)*q3)*dt;
  F(0, 0)= 1.0;
  F(0, 1)=-0.5*(pm -dp)*dt;
  F(0, 2)=-0.5*(qm -dq)*dt;
  F(0, 3)=-0.5*(rm -dr)*dt;
  F(0, 4)= 0.5*q1*dt;
  F(0, 5)= 0.5*q2*dt;
  F(0, 6)= 0.5*q3*dt;

  //x(1, 0) = q1 + 0.5*( (pm-dp)*q0 +(rm-dr)*q2 -(qm-dq)*q3)*dt;
  F(1, 0)= 0.5*(pm -dp)*dt;
  F(1, 1)= 1.0;
  F(1, 2)= 0.5*(rm -dr)*dt;
  F(1, 3)=-0.5*(qm -dq)*dt;
  F(1, 4)=-0.5*q0*dt;
  F(1, 5)= 0.5*q3*dt;
  F(1, 6)=-0.5*q2*dt;

  //x(2, 0) = q2 + 0.5*( (qm-dq)*q0 -(rm-dr)*q1 +(pm-dp)*q3)*dt; <-miss!
  F(2, 0)= 0.5*(qm -dq)*dt;
  F(2, 1)=-0.5*(rm -dr)*dt;
  F(2, 2)= 1.0;
  F(2, 3)= 0.5*(pm -dp)*dt;
  F(2, 4)=-0.5*q3*dt;
  F(2, 5)=-0.5*q0*dt;
  F(2, 6)= 0.5*q1*dt;
  
  //x(3, 0) = q3 + 0.5*( (rm-dr)*q0 +(qm-dq)*q1 -(pm-dp)*q2)*dt;
  F(3, 0)= 0.5*(rm -dr)*dt;
  F(3, 1)= 0.5*(qm -dq)*dt;
  F(3, 2)=-0.5*(pm -dp)*dt;
  F(3, 3)= 1.0;
  F(3, 4)= 0.5*q2*dt;
  F(3, 5)=-0.5*q1*dt;
  F(3, 6)=-0.5*q0*dt;
  
  //x(4, 0) = dp -betax*dp*dt;
  F(4, 0)= 0.0;
  F(4, 1)= 0.0;
  F(4, 2)= 0.0;
  F(4, 3)= 0.0;
  F(4, 4)= 1.0 - betax*dt;
  F(4, 5)= 0.0;
  F(4, 6)= 0.0;
  
  //x(5, 0) = dq - betay*dq*dt;
  F(5, 0)= 0.0;
  F(5, 1)= 0.0;
  F(5, 2)= 0.0;
  F(5, 3)= 0.0;
  F(5, 4)= 0.0;
  F(5, 5)= 1.0 - betay*dt;
  F(5, 6)= 0.0;
  
  //x(6, 0) = dr -betaz*dr*dt;
  F(6, 0)= 0.0;
  F(6, 1)= 0.0;
  F(6, 2)= 0.0;
  F(6, 3)= 0.0;
  F(6, 4)= 0.0;
  F(6, 5)= 0.0;
  F(6, 6)= 1.0 - betaz*dt;
  
  return 0;
}

//Make Jacobian matrix H
uint8_t H_jacobian( Matrix<float, 6, 7> &H, 
                    Matrix<float, 7, 1> x, 
                    float g, 
                    float mn, 
                    float me,
                    float md)
{
  float q0 = x(0, 0);
  float q1 = x(1, 0);
  float q2 = x(2, 0);
  float q3 = x(3, 0);

  //z(0, 0) = 2.0*(q1*q3 - q0*q2)*g;
  H(0, 0) =-2.0*q2*g;
  H(0, 1) = 2.0*q3*g;
  H(0, 2) =-2.0*q0*g;
  H(0, 3) = 2.0*q1*g;
  H(0, 4) = 0.0;
  H(0, 5) = 0.0;
  H(0, 6) = 0.0;
  
  //z(1, 0) = 2.0*(q2*q3 + q0*q1)*g;
  H(1, 0) = 2.0*q1*g;
  H(1, 1) = 2.0*q0*g;
  H(1, 2) = 2.0*q3*g;
  H(1, 3) = 2.0*q2*g;
  H(1, 4) = 0.0;
  H(1, 5) = 0.0;
  H(1, 6) = 0.0;
  
  //z(2, 0) = (q0*q0 - q1*q1 - q2*q2 + q3*q3)*g ;
  H(2, 0) = 2.0*q0*g;
  H(2, 1) =-2.0*q1*g;
  H(2, 2) =-2.0*q2*g;
  H(2, 3) = 2.0*q3*g;
  H(2, 4) = 0.0;
  H(2, 5) = 0.0;
  H(2, 6) = 0.0;
  
  //z(3, 0) = (q0*q0 + q1*q1 - q2*q2 - q3*q3)*mn + 2.0*(q1*q2 + q0*q3)*me + 2.0*(q1*q3 - q0*q2)*md;
  H(3, 0) = 2.0*( q0*mn + q3*me - q2*md);
  H(3, 1) = 2.0*( q1*mn + q2*me + q3*md);
  H(3, 2) = 2.0*(-q2*mn + q1*me - q0*md);
  H(3, 3) = 2.0*(-q3*mn + q0*me + q1*md);
  H(3, 4) = 0.0;
  H(3, 5) = 0.0;
  H(3, 6) = 0.0;
  
  //z(4, 0) = 2.0*(q1*q2 - q0*q3)*mn + (q0*q0 - q1*q1 + q2*q2 -q3*q3)*me+ 2.0*(q2*q3 + q0*q1)*md;
  H(4, 0) = 2.0*(-q3*mn + q0*me + q1*md);
  H(4, 1) = 2.0*( q2*mn - q1*me + q0*md);
  H(4, 2) = 2.0*( q1*mn + q2*me + q3*md);
  H(4, 3) = 2.0*(-q0*mn - q3*me + q2*md);
  H(4, 4) = 0.0;
  H(4, 5) = 0.0;
  H(4, 6) = 0.0;
  
  //z(5, 0) = 2.0*(q1*q3 + q0*q2)*mn + 2.0*(q2*q3 - q0*q1)*me + (q0*q0 - q1*q1 - q2*q2 + q3*q3)*md;
  H(5, 0) = 2.0*( q2*mn - q1*me + q0*md);
  H(5, 1) = 2.0*( q3*mn - q0*me - q1*md);
  H(5, 2) = 2.0*( q0*mn + q3*me - q2*md);
  H(5, 3) = 2.0*( q1*mn + q2*me + q3*md);
  H(5, 4) = 0.0;
  H(5, 5) = 0.0;
  H(5, 6) = 0.0;
  
  return 0;
}

//Extended Kalman Filter
uint8_t ekf( Matrix<float, 7, 1> &xe,
             Matrix<float, 7, 1> &xp,
             Matrix<float, 7, 7> &P,
             Matrix<float, 6, 1> z,
             Matrix<float, 3, 1> omega,
             Matrix<float, 6, 6> Q, 
             Matrix<float, 6, 6> R, 
             Matrix<float, 7, 6> G,
             Matrix<float, 3, 1> beta,
             float dt)
{
  Matrix<float, 7, 7> F;
  Matrix<float, 6, 7> H;
  Matrix<float, 6, 6> Den;
  //Matrix<float, 6, 6> I6=MatrixXf::Identity(6,6);
  Matrix<float, 7, 6> K;
  Matrix<float, 6, 1> zbar;
  float mag;

  //Update
  H_jacobian(H, xp, GRAV, MN, ME, MD);
  Den = H * P * H.transpose() + R;
  //PartialPivLU< Matrix<float, 6, 6> > dec(Den);
  //Den = dec.solve(I6);
  K = P * H.transpose() * Den.inverse();
  observation_equation(xp, zbar, GRAV, MN, ME, MD);
  xe = xp + K*(z - zbar);
  P = P - K*H*P;

  //Predict
  state_equation(xe, omega, beta, dt, xp);
  F_jacobian(F, xe, omega, beta, dt);
  P = F*P*F.transpose() + G*Q*G.transpose();

  mag=sqrt(xe(0,0)*xe(0,0) + xe(1,0)*xe(1,0) + xe(2,0)*xe(2,0) + xe(3,0)*xe(3,0));
  xe(0,0)/=mag;
  xe(1,0)/=mag;
  xe(2,0)/=mag;
  xe(3,0)/=mag;
  mag=sqrt(xp(0,0)*xp(0,0) + xp(1,0)*xp(1,0) + xp(2,0)*xp(2,0) + xp(3,0)*xp(3,0));
  xp(0,0)/=mag;
  xp(1,0)/=mag;
  xp(2,0)/=mag;
  xp(3,0)/=mag;


  return 0;
}
