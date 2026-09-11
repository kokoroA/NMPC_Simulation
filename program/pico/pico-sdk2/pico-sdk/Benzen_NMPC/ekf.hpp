#ifndef EKF_H
#define EKF_H

#include <stdio.h>
#include <iostream>
#include <random>
#include "pico/stdlib.h"
#include <Eigen/Dense>
#include <unistd.h>
#include <math.h>

#define GRAV (9.80665)

using Eigen::MatrixXd;
using Eigen::MatrixXf;
using Eigen::Matrix;
using Eigen::PartialPivLU;
using namespace Eigen;

extern float MN,ME,MD;
 
//Kalman filter for alt and line trace
extern Matrix<float, 2 ,2> Sigma_Yn_est;
extern Matrix<float, 2,2> Sigma_Yn_pre;
extern Matrix<float, 1,2> observation_mat;
extern Matrix<float, 2,1> observation_mat_transposed;
extern Matrix<float, 2,1> control_mat;
extern Matrix<float, 2,2> unit_mat;
extern Matrix<float, 2,2> system_mat;
extern Matrix<float, 1, 1> Kal_element;
extern Matrix<float, 2, 1> K;
extern Matrix<float, 1, 1> R_mat;
// extern Matrix<float, 2, 1> R;
extern Matrix<float,1,1> Kal_element_inv;
extern Matrix<float, 2,2> last_Sigma_Yn_pre;
extern Matrix<float, 2,1> mu_Yn_pre;
extern Matrix<float, 2,1> last_mu_Yn_pre;
extern Matrix<float, 2,1> mu_Yn_est;
extern Matrix<float, 2,2> Q_mat;
extern Matrix<float, 1,1> Q_k_mat;
extern Matrix<float, 1,1> u_n;
extern Matrix<float, 1,1> yn_mat;
extern Matrix<float, 1 ,1> k_inv;
extern Matrix<float, 1,1> mu_Yn_est_par;
extern Matrix<float, 2,1> mu_Yn_est_par2;
extern float Xn_est_1,Xn_est_2,Xn_est_3,u;
extern float integral,integral_v,differential,differential_v;

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
             float dt);

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
                  );

float Kalman_PID(float observe_y,float Ax);
float Kalman_holizontal(float camera_y,float camera_psi,float deltaP,float deltaR,float deltaPhi);
float alt_PID(float ref_alt);
void Kalman_init(void);           
#endif
