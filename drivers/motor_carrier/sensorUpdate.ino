void sensorUpdate()
{
  // Read Encoder values
  RPM1 = (((encoder1.getRawCount())*(1000.0))/(50.0*40.0)*60.0);
  RPM2 = (((encoder2.getRawCount())*(1000.0))/(50.0*40.0)*60.0);

  encoder1.resetCounter(0);
  encoder2.resetCounter(0);

  // Get orientation data
  // bno055_read_quaternion_wxyz();
  bno055_read_euler_hrp(&eulerData);
  delay(1);
  float transformed_heading = fmod(90 - float(eulerData.h)/16.00 + 360.0, 360.0);

  enu_heading = (uint16_t)(transformed_heading * 100.0);

  // Get gyro data
  // bno055_read_gyro_xyz(&gyroData);

  // Get accel data
  // bno055_read_linear_accel_xyz(&accelData);

  // Get mag data
  // bno055_read_mag_xyz(&magData);

}
