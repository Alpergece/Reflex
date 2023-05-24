#include <KalmanFilter.h>

#include <cmath>


KalmanFilter::KalmanFilter(
    int state_dim,
    int obs_dim,
    int control_dim,
    Eigen::VectorXd init_state,
    float init_noise_var,
    float process_noise_var,
    float obs_noise_var
)
{
    //Initialize dimensions
    state_dim_ = state_dim;
    control_dim_ = control_dim;
    obs_dim_ = obs_dim;

    //Initialize noise variances.
    init_noise_var_ = init_noise_var;
    process_noise_var_ = process_noise_var;
    obs_noise_var_ = obs_noise_var;

    init_state_ = init_state;

    //Initialize Matrices 

    //State Matrix
    X_ = init_state_;
    //State - transition Matrix (state_dim_ X state_dim_) : x(t) = A * x(t - 1) + w(t - 1)
    A_.setIdentity(state_dim_, state_dim_);
    //Control Matrix (state_dim_ X control_dim_)
    B_.setZero(state_dim_, control_dim_);
    //Initial uncertainty (state_dim_ X state_dim_)
    P_.setIdentity(state_dim_, state_dim_);
    P_ = P_.eval() * init_noise_var_;
    //Covariance of the process noise (state_dim_ X state_dim_)
    Q_.setIdentity(state_dim_, state_dim_);
    Q_ = Q_.eval() * process_noise_var_;
    //Observation matrix (obs_dim_ X state_dim_) - positions are observed and not velocities
    H_.setIdentity(obs_dim_, state_dim_); //TODO
    //Covariance of the observation noise (obs_dim_ X obs_dim_)
    R_.setIdentity(obs_dim_, obs_dim_);
    R_ = R_.eval() * obs_noise_var_;
    //Kalman Gain (state_dim_ X obs_dim_)
    K_.setZero(state_dim_, obs_dim_);
    //Identity matrix (state_dim_ X state_dim_)
    I_.setIdentity(state_dim_, state_dim_);

}

void KalmanFilter::setAdt(float dt)
{
    A_ << 1, 0, 0, dt, 0, 0,
        0, 1, 0, 0, dt, 0,
        0, 0, 1, 0, 0, dt,
        0, 0, 0, 1, 0, 0,
        0, 0, 0, 0, 1, 0,
        0, 0, 0, 0, 0, 1;
}

void KalmanFilter::setA(Eigen::MatrixXd A)
{
    A_ = A;
}

void KalmanFilter::setH(Eigen::MatrixXd H)
{
    H_ = H;
}

void KalmanFilter::setB(Eigen::MatrixXd B)
{
    B_ = B;
}

Eigen::VectorXd KalmanFilter::predict(float dt)
{
    //Predict state

    Eigen::VectorXd U_;
    U_.setZero(control_dim_);
    setAdt(dt);
    X_ = A_ * X_.eval();// +B_ * U_;

    //Predict error covariance
    P_ = A_ * P_.eval() * A_.transpose() + Q_;

    return X_;
}

Eigen::VectorXd KalmanFilter::update(Eigen::VectorXd Z)
{
    //Update Kalman gain
    K_ = P_ * H_.transpose() * (H_ * P_ * H_.transpose() + R_).inverse();
   
    //Update state estimate using new observation Z
    X_ = X_.eval() + K_ * (Z - H_ * X_.eval());

    //Update uncertainty covariance
    P_ = (I_ - K_ * H_) * P_.eval();
    return X_;
}

void KalmanFilter::initializeKalman(Eigen::VectorXd init_state)
{
    K_.setZero(state_dim_, obs_dim_);
    X_ = init_state; //TODO
    P_.setIdentity(state_dim_, state_dim_);
    P_ = P_.eval() * init_noise_var_;
}

ConstantVelocityKalmanFilter::ConstantVelocityKalmanFilter(float dt, int state_dim, int obs_dim, int control_dim, Eigen::VectorXd init_state,
    float init_noise_var,
    float process_noise_var,
    float obs_noise_var) : KalmanFilter(state_dim, obs_dim, control_dim, init_state, init_noise_var, process_noise_var, obs_noise_var)
{
    Eigen::MatrixXd H = Eigen::MatrixXd::Identity(obs_dim,state_dim);
    setH(H);

    //State - transition matrix(6x6) : x(t) = A * x(t - 1) + w(t - 1) use set Adt function
    Eigen::MatrixXd A;
    A.resize(6, 6);
    A << 1, 0, 0, dt, 0, 0,
        0, 1, 0, 0, dt, 0,
        0, 0, 1, 0, 0, dt,
        0, 0, 0, 1, 0, 0,
        0, 0, 0, 0, 1, 0,
        0, 0, 0, 0, 0, 1;

    setA(A);

    //Control matrix(6x3)
    Eigen::MatrixXd B;
    B.resize(6, 3);
    B << 1 / 2.0 * pow(dt, 2), 0, 0,
        0, 1 / 2.0 * pow(dt, 2), 0,
        0, 0, 1 / 2.0 * pow(dt, 2),
        0, 0, 0,
        0, 0, 0,
        0, 0, 0;
    
    setB(B);

}