#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "ICM_20948.h"
#include "i2c_wrapper.hpp"


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("icm20948");
  auto logger = node->get_logger();
  int i2c_bus = node->declare_parameter("i2c_bus", 0);
  int i2c_address = node->declare_parameter("i2c_address", ICM_20948_I2C_ADDR_AD1);
  
  ICM_20948_I2CDEV icm20948(i2c_bus, i2c_address);
  icm20948.enableDebugging(&std::cout); // Enable debug messages to the console
  auto result = icm20948.checkID();
  if (result != ICM_20948_Stat_Ok) {
    RCLCPP_ERROR(logger, "Failed to connect to ICM-20948: %d", result);
    return -1;
  }
  RCLCPP_INFO(logger, "Successfully connected to ICM-20948");
  bool success = true;
  success &= (icm20948.initializeDMP() == ICM_20948_Stat_Ok);
  success &= (icm20948.enableDMPSensor(INV_ICM20948_SENSOR_ORIENTATION) == ICM_20948_Stat_Ok);

  // Enable any additional sensors / features
  success &= (icm20948.enableDMPSensor(INV_ICM20948_SENSOR_RAW_GYROSCOPE) == ICM_20948_Stat_Ok);
  success &= (icm20948.enableDMPSensor(INV_ICM20948_SENSOR_RAW_ACCELEROMETER) == ICM_20948_Stat_Ok);
  success &= (icm20948.enableDMPSensor(INV_ICM20948_SENSOR_MAGNETIC_FIELD_UNCALIBRATED) == ICM_20948_Stat_Ok);

  // Configuring DMP to output data at multiple ODRs:
  // DMP is capable of outputting multiple sensor data at different rates to FIFO.
  // Setting value can be calculated as follows:
  // Value = (DMP running rate / ODR ) - 1
  // E.g. For a 5Hz ODR rate when DMP is running at 55Hz, value = (55/5) - 1 = 10.
  success &= (icm20948.setDMPODRrate(DMP_ODR_Reg_Quat9, 0) == ICM_20948_Stat_Ok); // Set to the maximum
  success &= (icm20948.setDMPODRrate(DMP_ODR_Reg_Accel, 0) == ICM_20948_Stat_Ok); // Set to the maximum
  success &= (icm20948.setDMPODRrate(DMP_ODR_Reg_Gyro, 0) == ICM_20948_Stat_Ok); // Set to the maximum
  //success &= (icm20948.setDMPODRrate(DMP_ODR_Reg_Gyro_Calibr, 0) == ICM_20948_Stat_Ok); // Set to the maximum
  success &= (icm20948.setDMPODRrate(DMP_ODR_Reg_Cpass, 0) == ICM_20948_Stat_Ok); // Set to the maximum
  //success &= (icm20948.setDMPODRrate(DMP_ODR_Reg_Cpass_Calibr, 0) == ICM_20948_Stat_Ok); // Set to the maximum

  // Enable the FIFO
  success &= (icm20948.enableFIFO() == ICM_20948_Stat_Ok);

  // Enable the DMP
  success &= (icm20948.enableDMP() == ICM_20948_Stat_Ok);

  // Reset DMP
  success &= (icm20948.resetDMP() == ICM_20948_Stat_Ok);

  // Reset FIFO
  success &= (icm20948.resetFIFO() == ICM_20948_Stat_Ok);
  if (!success) {
    RCLCPP_ERROR(logger, "Failed to initialize DMP");
    return -1;
  }
  RCLCPP_INFO(logger, "Successfully initialized DMP");

int count = 10;
icm_20948_DMP_data_t data;
while (count >0)
{
  // Read any DMP data waiting in the FIFO
  // Note:
  //    readDMPdataFromFIFO will return ICM_20948_Stat_FIFONoDataAvail if no data is available.
  //    If data is available, readDMPdataFromFIFO will attempt to read _one_ frame of DMP data.
  //    readDMPdataFromFIFO will return ICM_20948_Stat_FIFOIncompleteData if a frame was present but was incomplete
  //    readDMPdataFromFIFO will return ICM_20948_Stat_Ok if a valid frame was read.
  //    readDMPdataFromFIFO will return ICM_20948_Stat_FIFOMoreDataAvail if a valid frame was read _and_ the FIFO contains more (unread) data.
  icm20948.readDMPdataFromFIFO(&data);

  if ((icm20948.status == ICM_20948_Stat_Ok) || (icm20948.status == ICM_20948_Stat_FIFOMoreDataAvail)) // Was valid data available?
  {
    //SERIAL_PORT.print(F("Received data! Header: 0x")); // Print the header in HEX so we can see what data is arriving in the FIFO
    //if ( data.header < 0x1000) SERIAL_PORT.print( "0" ); // Pad the zeros
    //if ( data.header < 0x100) SERIAL_PORT.print( "0" );
    //if ( data.header < 0x10) SERIAL_PORT.print( "0" );
    //SERIAL_PORT.println( data.header, HEX );

    if ((data.header & DMP_header_bitmap_Quat9) > 0) // We have asked for orientation data so we should receive Quat9
    {
      RCLCPP_INFO(logger, "Received Quat9 data! Header: 0x%04X", data.header);
      // Q0 value is computed from this equation: Q0^2 + Q1^2 + Q2^2 + Q3^2 = 1.
      // In case of drift, the sum will not add to 1, therefore, quaternion data need to be corrected with right bias values.
      // The quaternion data is scaled by 2^30.

      //SERIAL_PORT.printf("Quat9 data is: Q1:%ld Q2:%ld Q3:%ld Accuracy:%d\r\n", data.Quat9.Data.Q1, data.Quat9.Data.Q2, data.Quat9.Data.Q3, data.Quat9.Data.Accuracy);

      // Scale to +/- 1
      double q1 = ((double)data.Quat9.Data.Q1) / 1073741824.0; // Convert to double. Divide by 2^30
      double q2 = ((double)data.Quat9.Data.Q2) / 1073741824.0; // Convert to double. Divide by 2^30
      double q3 = ((double)data.Quat9.Data.Q3) / 1073741824.0; // Convert to double. Divide by 2^30
      double q0 = sqrt(1.0 - ((q1 * q1) + (q2 * q2) + (q3 * q3)));
      RCLCPP_INFO(logger, "Quat9 data is: Q0:%f Q1:%f Q2:%f Q3:%f Accuracy:%d", q0, q1, q2, q3, data.Quat9.Data.Accuracy);
      count--;
    }
  }

  if (icm20948.status != ICM_20948_Stat_FIFOMoreDataAvail) // If more data is available then we should read it right away - and not delay
  {
    usleep(10000); // 10 ms delay before checking for more data. This is to prevent us from reading the FIFO too frequently if no data is available, which can cause performance issues. If the DMP ODR is set very low, you may want to increase this delay.
  }
}

  return 0;
}
