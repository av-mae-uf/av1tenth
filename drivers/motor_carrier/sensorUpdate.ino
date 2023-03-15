void sensorUpdate()
{
  // Read Encoder values
  // encoder1.getRawCount();
  // encoder1.getCountPerSecond();
  // RPM1 = (((encoder1.getRawCount())*(1000.0))/(50.0*80.0));
  RPM1 = 1.23;
  RPM2 = 1.76;

  // RPM2 = (((encoder2.getRawCount())*(1000.0))/(50.0*80.0));

  // Get orientation data
  // bno055_read_quaternion_wxyz();
  bno055_read_euler_hrp(&eulerData);
  delay(1);
  float transformed_heading = fmod(270.0 - float(eulerData.h)/16.00 + 360.0, 360.0);

  enu_heading = (uint16_t)(transformed_heading * 100.0);

  // Get gyro data
  // bno055_read_gyro_xyz(&gyroData);

  // Get accel data
  // bno055_read_linear_accel_xyz(&accelData);

  // Get mag data
  // bno055_read_mag_xyz(&magData);

}
