// ------------------------------------------------------------------------------- //
// Advanced Kalman Filtering and Sensor Fusion Course - Extended Kalman Filter
//
// ####### STUDENT FILE #######
//
// Usage:
// -Rename this file to "kalmanfilter.cpp" if you want to use this code.

#include "kalmanfilter.h"
#include "utils.h"

// -------------------------------------------------- //
// YOU CAN USE AND MODIFY THESE CONSTANTS HERE
constexpr double ACCEL_STD = 1.0;
constexpr double GYRO_STD = 0.01/180.0 * M_PI;
constexpr double INIT_VEL_STD = 10.0;
constexpr double INIT_PSI_STD = 45.0/180.0 * M_PI;
constexpr double GPS_POS_STD = 3.0;
constexpr double LIDAR_RANGE_STD = 3.0;
constexpr double LIDAR_THETA_STD = 0.02;
// -------------------------------------------------- //

void KalmanFilter::handleLidarMeasurements(const std::vector<LidarMeasurement>& dataset, const BeaconMap& map)
{
    // Assume No Correlation between the Measurements and Update Sequentially
    for(const auto& meas : dataset) {handleLidarMeasurement(meas, map);}
}

void KalmanFilter::handleLidarMeasurement(LidarMeasurement meas, const BeaconMap& map)
{
    if (isInitialised())
    {
        VectorXd state = getState();
        MatrixXd cov = getCovariance();

        // Implement The Kalman Filter Update Step for the Lidar Measurements in the 
        // section below.
        // HINT: use the wrapAngle() function on angular values to always keep angle
        // values within correct range, otherwise strange angle effects might be seen.
        // HINT: You can use the constants: LIDAR_RANGE_STD, LIDAR_THETA_STD
        // HINT: The mapped-matched beacon position can be accessed by the variables
        // map_beacon.x and map_beacon.y
        // ----------------------------------------------------------------------- //
        // Variable identifiying

        double p_x = state(0);
        double p_y = state(1);
        double psi = state(2);
        double VehSpeed = state(3);

        BeaconData map_beacon = map.getBeaconWithId(meas.id); // Match Beacon with built in Data Association Id
        if (meas.id != -1 && map_beacon.id != -1)
        {          
            VectorXd z_current = Vector2d::Zero();
            VectorXd z_estimate = Vector2d::Zero();

            z_current << meas.range, meas.theta; // current data matrix is identified

            // The map matched beacon positions can be accessed using: map_beacon.x AND map_beacon.y
            double range_estimate = sqrt((map_beacon.x-p_x)*(map_beacon.x-p_x) + (map_beacon.y-p_y)*((map_beacon.y-p_y)));
            double theta_estimate = atan2((map_beacon.y-p_y),(map_beacon.x-p_x)) - psi;
            
            z_estimate << range_estimate, theta_estimate;
            z_estimate(1) = wrapAngle(z_estimate(1));

            VectorXd z_error = z_current - z_estimate;
            z_error(1) = wrapAngle(z_error(1));

            // Jacobian Calculating.
            MatrixXd Jacobian_H = MatrixXd(2,4);
            Jacobian_H << (p_x - map_beacon.x)/range_estimate, (p_y - map_beacon.y)/range_estimate,0,0, (map_beacon.y - p_y)/range_estimate/range_estimate, (p_x - map_beacon.x)/range_estimate/range_estimate,-1,0;
            
            // Standart deviation terms adding to Noise Matrix.
            const double range_std = LIDAR_RANGE_STD*LIDAR_RANGE_STD;
            const double theta_std = LIDAR_THETA_STD*LIDAR_THETA_STD;           
            
            MatrixXd Jacobian_R = Matrix2d::Zero();
            Jacobian_R(0,0) = range_std;
            Jacobian_R(1,1) = theta_std;           

            MatrixXd Jacobian_S = Jacobian_H * cov * Jacobian_H.transpose() + Jacobian_R;             
            MatrixXd KalmanGain = cov * Jacobian_H.transpose()* Jacobian_S.inverse();

            
            state = state + KalmanGain*z_error;
            cov = (Matrix4d::Identity() - KalmanGain*Jacobian_H) * cov;

        }
        // ----------------------------------------------------------------------- //
        
        setState(state);
        setCovariance(cov);
    }
}

void KalmanFilter::predictionStep(GyroMeasurement gyro, double dt)
{
    if (isInitialised())
    {
        VectorXd state = getState();
        MatrixXd cov = getCovariance();

        // Implement The Kalman Filter Prediction Step for the system in the  
        // section below.
        // HINT: Assume the state vector has the form [PX, PY, PSI, V].
        // HINT: Use the Gyroscope measurement as an input into the prediction step.
        // HINT: You can use the constants: ACCEL_STD, GYRO_STD
        // HINT: use the wrapAngle() function on angular values to always keep angle
        // values within correct range, otherwise strange angle effects might be seen.
        // ----------------------------------------------------------------------- //
        // Variable identifiying
        double p_x = state(0);
        double p_y = state(1);
        double psi = state(2);
        double VehSpeed = state(3);

        // Update state variables.
        double p_x_new = p_x + dt * VehSpeed * cos(psi);
        double p_y_new = p_y + dt * VehSpeed * sin(psi);
        double psi_new = wrapAngle(psi + dt * gyro.psi_dot);
        double VehSpeed_New = VehSpeed;
        
        state << p_x_new,p_y_new,psi_new,VehSpeed_New; // Updating with zero noise.

        // Jacobian Calculating.
        Matrix4d Jacobian_State = Matrix4d();
        Jacobian_State << 1,0,-dt*VehSpeed*sin(psi),dt*cos(psi), 0,1,dt*VehSpeed*cos(psi),dt*sin(psi), 0,0,1,0, 0,0,0,1;

        // Standart deviation terms adding to Noise Matrix.
        const double acc_std = ACCEL_STD*ACCEL_STD;
        const double gyro_std = GYRO_STD*GYRO_STD;

        Matrix4d Jacobian_Noise = Matrix4d::Zero();
        Jacobian_Noise(2,2) = dt*dt*gyro_std;
        Jacobian_Noise(3,3) = dt*dt*acc_std;

        cov = Jacobian_State * cov * Jacobian_State.transpose() + Jacobian_Noise; // Updating Covariance Matrix.

        // ----------------------------------------------------------------------- //

        setState(state);
        setCovariance(cov);
    } 
}

void KalmanFilter::handleGPSMeasurement(GPSMeasurement meas)
{
    // All this code is the same as the LKF as the measurement model is linear
    // so the EKF update state would just produce the same result.
    if(isInitialised())
    {
        VectorXd state = getState();
        MatrixXd cov = getCovariance();

        VectorXd z = Vector2d::Zero();
        MatrixXd H = MatrixXd(2,4);
        MatrixXd R = Matrix2d::Zero();

        z << meas.x,meas.y;
        H << 1,0,0,0,0,1,0,0;
        R(0,0) = GPS_POS_STD*GPS_POS_STD;
        R(1,1) = GPS_POS_STD*GPS_POS_STD;

        VectorXd z_hat = H * state;
        VectorXd y = z - z_hat;
        MatrixXd S = H * cov * H.transpose() + R;
        MatrixXd K = cov*H.transpose()*S.inverse();

        state = state + K*y;
        cov = (Matrix4d::Identity() - K*H) * cov;

        setState(state);
        setCovariance(cov);
    }
    else
    {
        VectorXd state = Vector4d::Zero();
        MatrixXd cov = Matrix4d::Zero();

        state(0) = meas.x;
        state(1) = meas.y;
        cov(0,0) = GPS_POS_STD*GPS_POS_STD;
        cov(1,1) = GPS_POS_STD*GPS_POS_STD;
        cov(2,2) = INIT_PSI_STD*INIT_PSI_STD;
        cov(3,3) = INIT_VEL_STD*INIT_VEL_STD;

        setState(state);
        setCovariance(cov);
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
        VectorXd state = getState(); // STATE VECTOR [X,Y,PSI,V,...]
        return VehicleState(state[0],state[1],state[2],state[3]);
    }
    return VehicleState();
}

void KalmanFilter::predictionStep(double dt){}
