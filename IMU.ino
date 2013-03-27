// The below code is not currently used.

// Below code found here - https://github.com/TKJElectronics/Example-Sketch-for-IMU-including-Kalman-filter/tree/master/IMU6DOF/ITG3205_ADXL345

// KalmanX

/* Kalman filter variables and constants */
const double Q_angleX = 0.001; // Process noise covariance for the accelerometer - Sw
const double Q_gyroX = 0.003; // Process noise covariance for the gyro - Sw
const double R_angleX = 0.03; // Measurement noise covariance - Sv

double angleX = 180; // The angle output from the Kalman filter
double biasX = 0; // The gyro bias calculated by the Kalman filter
double PX_00 = 0, PX_01 = 0, PX_10 = 0, PX_11 = 0;
double dtX, yX, SX;
double KX_0, KX_1;

double kalmanX(double newAngle, double newRate, double dtime) {
  // KasBot V2  -  Kalman filter module - http://www.arduino.cc/cgi-bin/yabb2/YaBB.pl?num=1284738418
  // See also http://www.x-firm.com/?page_id=145
  // with slightly modifications by Kristian Lauszus
  // See http://academic.csuohio.edu/simond/courses/eec644/kalman.pdf and 
  // http://www.cs.unc.edu/~welch/media/pdf/kalman_intro.pdf for more information
  dtX = dtime / 1000000; // Convert from microseconds to seconds

  // Discrete Kalman filter time update equations - Time Update ("Predict")
  // Update xhat - Project the state ahead
  angleX += dtX * (newRate - biasX);

  // Update estimation error covariance - Project the error covariance ahead
  PX_00 += -dtX * (PX_10 + PX_01) + Q_angleX * dtX;
  PX_01 += -dtX * PX_11;
  PX_10 += -dtX * PX_11;
  PX_11 += +Q_gyroX * dtX;

  // Discrete Kalman filter measurement update equations - Measurement Update ("Correct")
  // Calculate Kalman gain - Compute the Kalman gain
  SX = PX_00 + R_angleX;
  KX_0 = PX_00 / SX;
  KX_1 = PX_10 / SX;

  // Calculate angle and resting rate - Update estimate with measurement zk
  yX = newAngle - angleX;
  angleX += KX_0 * yX;
  biasX += KX_1 * yX;

  // Calculate estimation error covariance - Update the error covariance
  PX_00 -= KX_0 * PX_00;
  PX_01 -= KX_0 * PX_01;
  PX_10 -= KX_1 * PX_00;
  PX_11 -= KX_1 * PX_01;

  return angleX;
}


// Kalman Y

/* Kalman filter variables and constants */
const double Q_angleY = 0.001; // Process noise covariance for the accelerometer - Sw
const double Q_gyroY = 0.003; // Process noise covariance for the gyro - Sw
const double R_angleY = 0.03; // Measurement noise covariance - Sv

double angleY = 180; // The angle output from the Kalman filter
double biasY = 0; // The gyro bias calculated by the Kalman filter
double PY_00 = 0, PY_01 = 0, PY_10 = 0, PY_11 = 0;
double dtY, yY, SY;
double KY_0, KY_1;

double kalmanY(double newAngle, double newRate, double dtime) {
  // KasBot V2  -  Kalman filter module - http://www.arduino.cc/cgi-bin/yabb2/YaBB.pl?num=1284738418
  // See also http://www.x-firm.com/?page_id=145
  // with slightly modifications by Kristian Lauszus
  // See http://academic.csuohio.edu/simond/courses/eec644/kalman.pdf and 
  // http://www.cs.unc.edu/~welch/media/pdf/kalman_intro.pdf for more information
  dtY = dtime / 1000000; // Convert from microseconds to seconds

  // Discrete Kalman filter time update equations - Time Update ("Predict")
  // Update xhat - Project the state ahead
  angleY += dtY * (newRate - biasY);

  // Update estimation error covariance - Project the error covariance ahead
  PY_00 += -dtY * (PY_10 + PY_01) + Q_angleY * dtY;
  PY_01 += -dtY * PY_11;
  PY_10 += -dtY * PY_11;
  PY_11 += +Q_gyroY * dtY;

  // Discrete Kalman filter measurement update equations - Measurement Update ("Correct")
  // Calculate Kalman gain - Compute the Kalman gain
  SY = PY_00 + R_angleY;
  KY_0 = PY_00 / SY;
  KY_1 = PY_10 / SY;

  // Calculate angle and resting rate - Update estimate with measurement zk
  yY = newAngle - angleY;
  angleY += KY_0 * yY;
  biasY += KY_1 * yY;

  // Calculate estimation error covariance - Update the error covariance
  PY_00 -= KY_0 * PY_00;
  PY_01 -= KY_0 * PY_01;
  PY_10 -= KY_1 * PY_00;
  PY_11 -= KY_1 * PY_01;

  return angleY;
}
