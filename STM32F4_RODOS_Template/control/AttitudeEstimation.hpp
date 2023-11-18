#ifndef FLOATSAT_CONTROL_ATTITUDEESTIMATION_HPP_
#define FLOATSAT_CONTROL_ATTITUDEESTIMATION_HPP_

#include "rodos.h"
#include "matlib.h"
#include "math.h"

#include "../timestamp.hpp"
#include "../hardware/imu.hpp"

struct Attitude_Data 
{
	Quaternion attitude;
	Vector3D angularVelocity;
};

extern Topic<TimestampedData<Attitude_Data>> AttitudeDataTopic;

class QEKF
{
public:

	bool is_initialized = false;

private:

	// State
	Vector_F<4> X;
	// Covariance of state
	Matrix_F<4, 4> P;

	//-------- Prediction ----------//
	// Jakobians of prediction
	Matrix_F<4, 4> A;
	Matrix_F<4, 3> G;
	// Covariance of process noise
	Matrix_F<3, 3> Q;

	//--------Correction (accel) ----------//
	// Gain
	Matrix_F<4, 3> K_a;
	// Jakobian of mesurment prediction
	Matrix_F<3, 4> C_a;
	// Covariance innovation
	Matrix_F<3, 3> S_a;
	// Covariance of measurment noise
	Matrix_F<3, 3> R_a;
	// Measurment inovation
	Vector_F<3> v_a;
	// Measurment prediction
	Vector_F<3> z_a;

	//--------Correction (mag) ----------//
	// Gain
	Matrix_F<4, 1> K_m;
	// Jakobian of mesurment prediction
	Matrix_F<1, 4> C_m;
	// Covariance innovation
	Matrix_F<1, 1> S_m;
	// Covariance of measurment noise
	Matrix_F<1, 1> R_m;
	//Measurment inovation
	float v_yaw;
	// Measurment prediction
	float z_yaw;
	// Measurment
	float y_yaw;
	// Helper rotaions
	Matrix_F<3, 3> body2nav;
	Matrix_F<3, 3> nav2body;

	// Last timestamp
	float last_t;

	// Attitude data
	TimestampedData <Attitude_Data> data;

	// std of sensors
	Vector3D_F sigma_gyro = 1;
	Vector3D_F sigma_accel = 1;
	float sigma_yaw = 1;

public:

	QEKF();

	// @brief Calculation of initial orientation
	// @param imudata -> IMU data struct defined in imu.hpp (angularVelocity [rad/s], magneticField [gauss], acceleration [g])
	void init(IMUData& imudata);

	// @brief Orientation estimation
	// @param imudata -> IMU data struct defined in imu.hpp with timestamp of measurement.
	// @return Attitude data defined in AttitudeEstimation.hpp with timestamp of estimation.
	TimestampedData<Attitude_Data>& estimate(TimestampedData<IMUData>& imudata);

private:

	// @brief Propagation step using only gyro
	// @param gyro -> angluarvelocity vector in rad/s
	void propagate(Vector3D_F gyro);

	// @brief Update step using accelerometer
	// @param accel -> linearacceleration vector in g
	void update_accel(Vector3D_F a);

	// @brief Update step using magnetometer
	// @param mag -> magneticfieldstrength vector in gauss
	void update_mag(Vector3D_F m);

};


extern QEKF qekf;

#endif
