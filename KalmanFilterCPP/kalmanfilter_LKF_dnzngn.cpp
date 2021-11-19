// ------------------------------------------------------------------------------- //
// Advanced Kalman Filtering and Sensor Fusion Course - Linear Kalman Filter
//
// ####### STUDENT FILE #######
//
// Usage:
// -Rename this file to "kalmanfilter.cpp" if you want to use this code.

#include "kalmanfilter.h"
#include "utils.h"

// -------------------------------------------------- //
// YOU CAN USE AND MODIFY THESE CONSTANTS HERE
// Standart deviation initalize. This example give full trust to sensors. 
// When these values are changed, you can consider the sensors have low quality.
constexpr bool INIT_ON_FIRST_PREDICTION = true;
constexpr double INIT_POS_STD = 0;
constexpr double INIT_VEL_STD = 0;
constexpr double ACCEL_STD = 0;
constexpr double GPS_POS_STD = 3.0;
// -------------------------------------------------- //

void KalmanFilter::predictionStep(double dt)
{
    if (!isInitialised() && INIT_ON_FIRST_PREDICTION)
    {
        // Implement the State Vector and Covariance Matrix Initialisation in the
        // section below if you want to initialise the filter WITHOUT waiting for
        // the first measurement to occur. Make sure you call the setState() /
        // setCovariance() functions once you have generated the initial conditions.
        // Hint: Assume the state vector has the form [X,Y,VX,VY].
        // Hint: You can use the constants: INIT_POS_STD, INIT_VEL_STD
        // ----------------------------------------------------------------------- //
            VectorXd state = Vector4d::Zero();
            MatrixXd cov = Matrix4d::Zero();

            // Assume the initial position is (X,Y) = (0,0) m
            // Assume the initial velocity is 5 m/s at 45 degrees (VX,VY) = (5*cos(45deg),5*sin(45deg)) m/s

                                                             // This for initializing the state vector.   
            state << 0, 0, 5.0*cos(M_PI/4), 5.0*sin(M_PI/4); // [0 0 5*cos45 5*sin45]'
    
            const double init_pos_std = INIT_POS_STD;
            const double init_vel_std = INIT_VEL_STD;
            // No initial uncertanity.
            cov(0,0) = INIT_POS_STD*INIT_POS_STD; // P = E[(Initial-mean)*(Initial-mean)']
            cov(1,1) = INIT_POS_STD*INIT_POS_STD;
            cov(2,2) = INIT_VEL_STD*INIT_VEL_STD;
            cov(3,3) = INIT_VEL_STD*INIT_VEL_STD;
            
            setState(state);
            setCovariance(cov);
        // ----------------------------------------------------------------------- //
    }

    if (isInitialised())
    {
        VectorXd state = getState();
        MatrixXd cov = getCovariance();

        // Implement The Kalman Filter Prediction Step for the system in the  
        // section below.
        // Hint: You can use the constants: ACCEL_STD
        // ----------------------------------------------------------------------- //

        MatrixXd F = Matrix4d();
        F << 1,0,dt,0,0,1,0,dt,0,0,1,0,0,0,0,1; // 1 0 dt 0 
                                                // 0 1 0 dt
                                                // 0 0 1 0
                                                // 0 0 0 1

        MatrixXd Q = Matrix2d::Zero();
        // Acceleration is zero, constant velocity. When acceleration increases, stdev increases also.
        Q(0,0) = (ACCEL_STD*ACCEL_STD); // 0 0
        Q(1,1) = (ACCEL_STD*ACCEL_STD); // 0 0

        MatrixXd L = MatrixXd(4,2);
        L << (0.5*dt*dt),0,0,(0.5*dt*dt),dt,0,0,dt; // 0.5*dt² 0
                                                    // 0      0.5*dt²
                                                    // dt      0
                                                    // 0       dt

        state = F * state;
        cov = F * cov * F.transpose() + L * Q * L.transpose();

        // ----------------------------------------------------------------------- //

        setState(state);
        setCovariance(cov);
    }
}

void KalmanFilter::handleGPSMeasurement(GPSMeasurement meas)
{
    if(isInitialised())
    {
        VectorXd state = getState();
        MatrixXd cov = getCovariance();

        // Implement The Kalman Filter Update Step for the GPS Measurements in the 
        // section below.
        // Hint: Assume that the GPS sensor has a 3m (1 sigma) position uncertainty.
        // Hint: You can use the constants: GPS_POS_STD
        // ----------------------------------------------------------------------- // 

        VectorXd z = Vector2d();
        MatrixXd H = MatrixXd(2,4);
        MatrixXd R = Matrix2d::Zero();

        z << meas.x,meas.y;
        H << 1,0,0,0,0,1,0,0;
        R(0,0) = GPS_POS_STD*GPS_POS_STD;
        R(1,1) = GPS_POS_STD*GPS_POS_STD;

        VectorXd z_hat = H * state; // Measurement
        VectorXd y = z - z_hat; // Error
        MatrixXd S = H * cov * H.transpose() + R; // Innovation Covariance Matrix
        MatrixXd K = cov*H.transpose()*S.inverse(); // Kalman Gain

        state = state + K*y; // Updated  State Estimate
        cov = (MatrixXd::Identity(4,4) - K*H) * cov; // Updated Covariance Estimate

        // ----------------------------------------------------------------------- //

        setState(state);
        setCovariance(cov);
    }
    else
    {
        // Implement the State Vector and Covariance Matrix Initialisation in the
        // section below. Make sure you call the setState/setCovariance functions
        // once you have generated the initial conditions.
        // Hint: Assume the state vector has the form [X,Y,VX,VY].
        // Hint: You can use the constants: GPS_POS_STD, INIT_VEL_STD
        // ----------------------------------------------------------------------- //
        // When INIT_ON_FIRST_Prediction = false.
            VectorXd state = Vector4d::Zero();
            MatrixXd cov = Matrix4d::Zero();

            state(0) = meas.x;
            state(1) = meas.y;
            cov(0,0) = GPS_POS_STD*GPS_POS_STD;
            cov(1,1) = GPS_POS_STD*GPS_POS_STD;
            cov(2,2) = INIT_VEL_STD*INIT_VEL_STD;
            cov(3,3) = INIT_VEL_STD*INIT_VEL_STD;

            setState(state);
            setCovariance(cov);
        // ----------------------------------------------------------------------- //
    }        
}

Matrix2d KalmanFilter::getVehicleStatePositionCovariance()
{
    Matrix2d pos_cov = Matrix2d::Zero();
    MatrixXd cov = getCovariance();
    if (isInitialised() && cov.size() != 0){pos_cov << cov(0,0), cov(0,1), cov(1,0), cov(1,1);}
    return pos_cov;
}

VehicleState KalmanFilter::getVehicleState()
{
    if (isInitialised())
    {
        VectorXd state = getState(); // STATE VECTOR [X,Y,VX,VY]
        double psi = std::atan2(state[3],state[2]); // Direction
        double V = std::sqrt(state[2]*state[2] + state[3]*state[3]); // Velocity
        return VehicleState(state[0],state[1],psi,V);
    }
    return VehicleState();
}

void KalmanFilter::predictionStep(GyroMeasurement gyro, double dt){predictionStep(dt);}
void KalmanFilter::handleLidarMeasurements(const std::vector<LidarMeasurement>& dataset, const BeaconMap& map){}
void KalmanFilter::handleLidarMeasurement(LidarMeasurement meas, const BeaconMap& map){}

