#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

#include <Eigen/Dense>

class KalmanFilter
    {
    public:
        // constructor 
        KalmanFilter(
            int state_dim,
            int obs_dim,
            int control_dim,
            Eigen::VectorXd init_state,
            float init_noise_var,
            float process_noise_var,
            float obs_noise_var); 

        void setAdt(float dt);

        void setA(Eigen::MatrixXd);
        void setH(Eigen::MatrixXd);
        void setB(Eigen::MatrixXd);
        Eigen::VectorXd predict(float);
        Eigen::VectorXd update(Eigen::VectorXd);

        void initializeKalman(Eigen::VectorXd init_state);




    private:
        float dt_;

        float init_noise_var_;
        float process_noise_var_;
        float obs_noise_var_;

        int control_dim_;
        int state_dim_;
        int obs_dim_;

        Eigen::VectorXd init_state_;

        //State Matrix
        Eigen::VectorXd X_;
        //State - transition Matrix (state_dim_ X state_dim_) : x(t) = A * x(t - 1) + w(t - 1)
        Eigen::MatrixXd A_;
        //Control Matrix (state_dim_ X control_dim_)
        Eigen::MatrixXd B_;
        //Initial uncertainty (state_dim_ X state_dim_)
        Eigen::MatrixXd P_;
        //Covariance of the process noise (state_dim_ X state_dim_)
        Eigen::MatrixXd Q_;
        //Observation matrix (obs_dim_ X state_dim_) - positions are observed and not velocities
        Eigen::MatrixXd H_;
        //Covariance of the observation noise (obs_dim_ X obs_dim_)
        Eigen::MatrixXd R_;
        //Kalman Gain (state_dim_ X obs_dim_)
        Eigen::MatrixXd K_;
        //Identity matrix (state_dim_ X state_dim_)
        Eigen::MatrixXd I_;


    };

    class ConstantVelocityKalmanFilter : public KalmanFilter
    {
    public:
        ConstantVelocityKalmanFilter(float dt, int state_dim, int obs_dim, int control_dim, Eigen::VectorXd init_state, float init_noise_var, float process_noise_var, float obs_noise_var);

    };

#endif //KALMAN_FILTER_H