/*********************************************************************************
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2024 Woohyun Byun.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 ********************************************************************************/
/** @author Woohyun Byun - imbwh@cau.ac.kr
 *  @date 2024.09.29
 */ 
#ifndef __LKF_H__
#define __LKF_H__

#include <eigen3/Eigen/Dense>


namespace aims_fly
{

template <int n_states, int n_inputs, int n_measurements>
class LKF
{
    public:
        static const int nx = n_states;
        static const int nu = n_inputs;
        static const int nz = n_measurements;

        typedef Eigen::Matrix<double, nx, 1> x_t;
        typedef Eigen::Matrix<double, nu, 1> u_t;
        typedef Eigen::Matrix<double, nz, 1> z_t;

        typedef Eigen::Matrix<double, nx, nx> F_t;
        typedef Eigen::Matrix<double, nx, nu> G_t;
        typedef Eigen::Matrix<double, nx, nx> P_t;
        typedef Eigen::Matrix<double, nx, nx> Q_t;
        typedef Eigen::Matrix<double, nz, nz> R_t;
        typedef Eigen::Matrix<double, nz, nx> H_t;
        typedef Eigen::Matrix<double, nx, nz> K_t;

        struct esti_t
        {
            x_t x;
            P_t P;
            ros::Time t = ros::Time(0);
        };
            
        LKF() : F_(F_t::Identity()), G_(G_t::Zero()), H_(H_t::Zero()), 
                Q_(Q_t::Identity()), R_(R_t::Identity()) {};
        ~LKF() {};

        void setMatrices(const F_t &F, const G_t &G, const H_t &H);
        void setCovMatrices(const Q_t &Q, const R_t &R);

        esti_t init(const esti_t &esti, const double P0);
        esti_t correct(const esti_t &esti, const z_t &z);
        esti_t predict(const esti_t &esti, const u_t &u);

     private:
        F_t F_;
        G_t G_;
        H_t H_;
        Q_t Q_;
        R_t R_;

        K_t computeKalmanGain(const esti_t &esti);
        x_t predictState(const x_t &x, const u_t &u);
        P_t predictCov(const P_t &P);

    

};

template<int n_states, int n_inputs, int n_measurements>
void LKF<n_states, n_inputs, n_measurements>::setMatrices(const F_t &F, const G_t &G, const H_t &H)
{
    F_ = F;
    G_ = G;
    H_ = H;
}


template<int n_states, int n_inputs, int n_measurements>
void LKF<n_states, n_inputs, n_measurements>::setCovMatrices(const Q_t &Q, const R_t &R)
{
    Q_ = Q;  // arg of Q is Q * acc_sq (random variance in acceleration [(m/s^2)^2])
    R_ = R;  // measurement uncertainty
}


/** Initialize
 *  1.1. estimate uncertainty initial guess
 *  1.2. system state initial guess
 *  2. predict (not here but main)
 *  3. wait for new measure (not here but main)
 */
template<int n_states, int n_inputs, int n_measurements>
typename LKF<n_states, n_inputs, n_measurements>::esti_t
LKF<n_states, n_inputs, n_measurements>::init(const esti_t &esti, const double P0)
{   
    esti_t esti_0;
    esti_0.x = esti.x;
    esti_0.P = P0 * esti.P;
    return esti_0;
}

/** Update
 *  1. calculate the kalman gain 
 *  2.1. estimate the current state using the state update equation
 *  2.2. update the current estimate uncertainty
 */
template<int n_states, int n_inputs, int n_measurements>
typename LKF<n_states, n_inputs, n_measurements>::esti_t
LKF<n_states, n_inputs, n_measurements>::correct(const esti_t &esti, const z_t &z)
{
    esti_t up_esti;
    const K_t K = computeKalmanGain(esti);  // kalman gain
    
    const z_t y = z - (H_ * esti.x);               // innovation
    const P_t I = P_t::Identity();

    up_esti.x = esti.x + K*y;
    up_esti.P = (I-K*H_) * esti.P * (I-K*H_).transpose() + K*R_*K.transpose();
    return up_esti;
}

template<int n_states, int n_inputs, int n_measurements>
typename LKF<n_states, n_inputs, n_measurements>::K_t
LKF<n_states, n_inputs, n_measurements>::computeKalmanGain(const esti_t &esti)
{
    const R_t S = (H_ * esti.P * H_.transpose()) + R_;       // innovation covariance
    return esti.P * H_.transpose() * S.inverse();
}


/** Predict
 *  1.1. calculate the predicted state for the next iteration using system's dynamics
 *  1.2. extrapolate the estimate uncertainty 
 */

template<int n_states, int n_inputs, int n_measurements>
typename LKF<n_states, n_inputs, n_measurements>::esti_t
LKF<n_states, n_inputs, n_measurements>::predict(const esti_t &esti, const u_t &u)
{
    esti_t pred;
    pred.x = predictState(esti.x, u);
    pred.P = predictCov(esti.P);
    return pred;
}

template<int n_states, int n_inputs, int n_measurements>
typename LKF<n_states, n_inputs, n_measurements>::x_t 
LKF<n_states, n_inputs, n_measurements>::predictState(const x_t &x, const u_t &u)
{
    return F_ * x + G_ * u; 
}

template<int n_states, int n_inputs, int n_measurements>
typename LKF<n_states, n_inputs, n_measurements>::P_t
LKF<n_states, n_inputs, n_measurements>::predictCov(const P_t &P)
{
    return F_ * P * F_.transpose() + Q_;
}

};
#endif