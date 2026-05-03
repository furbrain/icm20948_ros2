#include <iostream>
#include <memory>
#include <rclcpp/logger.hpp>
#include <rclcpp/utilities.hpp>
#include "ICM_20948_DMP.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "ICM_20948.h"
#include "i2c_wrapper.hpp"

class ICM20948_Node : public rclcpp::Node
{
  private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_imu_;

  protected:
    int i2c_bus_;
    int i2c_address_;
    std::string frame_id_;
    int dmp_rate_ = 55;
    int odr_;
    int gyro_fss_;
    int acc_fss_;
    ICM_20948_ACCEL_CONFIG_FS_SEL_e acc_fss_enum_ = gpm4;
    ICM_20948_GYRO_CONFIG_1_FS_SEL_e gyro_fss_enum_ = dps2000;
    std::unique_ptr<ICM_20948_I2CDEV> icm20948_;

  public:
    ICM20948_Node() : Node("icm20948_node") {
      i2c_bus_ = this->declare_parameter("i2c_bus", 0);
      i2c_address_ = this->declare_parameter("i2c_address", ICM_20948_I2C_ADDR_AD1);
      frame_id_ = this->declare_parameter("frame_id", std::string("imu"));
      gyro_fss_ = this->declare_parameter("gyro_fss", 2000);
      acc_fss_ = this->declare_parameter("acc_fss", 4);
      odr_ = this->declare_parameter("odr", 50);
      check_params();
      icm20948_ = std::make_unique<ICM_20948_I2CDEV>(i2c_bus_, i2c_address_);
    
      icm20948_->enableDebugging(&std::cout); // Enable debug messages to the console

      usleep(100000); // Sleep for 10ms to allow the ICM-20948 to power up before we start communicating with it
      icm20948_->startupDefault(true);
      usleep(100000); // Sleep for 10ms to allow the ICM-20948 to complete its startup before we start communicating with it
      initialise();
      publisher_imu_ = this->create_publisher<sensor_msgs::msg::Imu>("imu", 10);
      timer_ = this->create_wall_timer(std::chrono::milliseconds(500/odr_), std::bind(&ICM20948_Node::readData, this));
    }

    void check_params() {
        RCLCPP_WARN(this->get_logger(), "Checking params: Accel FSS %d", acc_fss_);
        RCLCPP_WARN(this->get_logger(), "Checking params: Gyro FSS %d", gyro_fss_);
        RCLCPP_WARN(this->get_logger(), "Checking params: ODR %d", odr_);
      // Check if parameters are within valid ranges and log warnings if not
      switch (gyro_fss_) {
        case 250:
          gyro_fss_enum_ = dps250;
          break;
        case 500:
          gyro_fss_enum_ = dps500;
          break;
        case 1000:
          gyro_fss_enum_ = dps1000;
          break;
        case 2000:
          gyro_fss_enum_ = dps2000;
          break;
        default:
          RCLCPP_WARN(this->get_logger(), "Gyro FSS %d is invalid. Setting to default (2000).", gyro_fss_);
          gyro_fss_enum_ = dps2000;
          gyro_fss_ = 2000;
      }
      switch (acc_fss_) {
        case 2:
          acc_fss_enum_ = gpm2;
          break;
        case 4:
          acc_fss_enum_ = gpm4;
          break;
        case 8:
          acc_fss_enum_ = gpm8;
          break;
        case 16:
          acc_fss_enum_ = gpm16;
          break;
        default:
          RCLCPP_WARN(this->get_logger(), "Accel FSS %d is invalid. Setting to default (4).", acc_fss_);
          acc_fss_enum_ = gpm4;
          acc_fss_ = 4;
      }
      if (odr_ < 1 || odr_ > 225) {
        RCLCPP_WARN(this->get_logger(), "ODR %d is out of range (1-225). Setting to default (50).", odr_);
        odr_ = 50;
      } 
    }

    void initialise() {
      auto result = icm20948_->checkID();
      if (result != ICM_20948_Stat_Ok) {
        RCLCPP_ERROR(this->get_logger(), "Failed to connect to ICM-20948: %d", result);
        throw std::runtime_error("Failed to connect to ICM-20948");
      }
      RCLCPP_INFO(this->get_logger(), "Successfully connected to ICM-20948");
      bool success = true;
      success &= (icm20948_->initializeDMP(acc_fss_enum_,gyro_fss_enum_) == ICM_20948_Stat_Ok);
      success &= (icm20948_->enableDMPSensor(INV_ICM20948_SENSOR_ROTATION_VECTOR) == ICM_20948_Stat_Ok);

      // Enable any additional sensors / features
      success &= (icm20948_->enableDMPSensor(INV_ICM20948_SENSOR_GYROSCOPE) == ICM_20948_Stat_Ok);
      success &= (icm20948_->enableDMPSensor(INV_ICM20948_SENSOR_ACCELEROMETER) == ICM_20948_Stat_Ok);
      success &= (icm20948_->enableDMPSensor(INV_ICM20948_SENSOR_GEOMAGNETIC_FIELD) == ICM_20948_Stat_Ok);
      success &= (icm20948_->enableDMPSensor(INV_ICM20948_SENSOR_MAGNETIC_FIELD_UNCALIBRATED) == ICM_20948_Stat_Ok);
      //success &= (icm20948_->enableDMPSensor(INV_ICM20948_SENSOR_GYROSCOPE_UNCALIBRATED) == ICM_20948_Stat_Ok);

      //Set ODRs
      int odr_divisor = (dmp_rate_ / odr_) - 1;
      success &= (icm20948_->setDMPODRrate(DMP_ODR_Reg_Quat9, odr_divisor) == ICM_20948_Stat_Ok); // Set to the maximum
      success &= (icm20948_->setDMPODRrate(DMP_ODR_Reg_Accel, odr_divisor) == ICM_20948_Stat_Ok); // Set to the maximum
      success &= (icm20948_->setDMPODRrate(DMP_ODR_Reg_Gyro, odr_divisor) == ICM_20948_Stat_Ok); // Set to the maximum
      success &= (icm20948_->setDMPODRrate(DMP_ODR_Reg_Gyro_Calibr, odr_divisor) == ICM_20948_Stat_Ok); // Set to the maximum
      success &= (icm20948_->setDMPODRrate(DMP_ODR_Reg_Cpass, odr_divisor) == ICM_20948_Stat_Ok); // Set to the maximum
      success &= (icm20948_->setDMPODRrate(DMP_ODR_Reg_Cpass_Calibr, odr_divisor) == ICM_20948_Stat_Ok); // Set to the maximum
      // Enable the FIFO
      success &= (icm20948_->enableFIFO() == ICM_20948_Stat_Ok);

      // Enable the DMP
      success &= (icm20948_->enableDMP() == ICM_20948_Stat_Ok);

      // Reset DMP
      success &= (icm20948_->resetDMP() == ICM_20948_Stat_Ok);

      // Reset FIFO
      success &= (icm20948_->resetFIFO() == ICM_20948_Stat_Ok);
      if (!success) {
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize DMP");
        throw std::runtime_error("Failed to initialize DMP");
      }
      RCLCPP_INFO(this->get_logger(), "Successfully initialized DMP");
    }

    void readData() {
    // Read any DMP data waiting in the FIFO
    // Note:
    //    readDMPdataFromFIFO will return ICM_20948_Stat_FIFONoDataAvail if no data is available.
    //    If data is available, readDMPdataFromFIFO will attempt to read _one_ frame of DMP data.
    //    readDMPdataFromFIFO will return ICM_20948_Stat_FIFOIncompleteData if a frame was present but was incomplete
    //    readDMPdataFromFIFO will return ICM_20948_Stat_Ok if a valid frame was read.
    //    readDMPdataFromFIFO will return ICM_20948_Stat_FIFOMoreDataAvail if a valid frame was read _and_ the FIFO contains more (unread) data.
    icm_20948_DMP_data_t data;
  
    do {
      icm20948_->readDMPdataFromFIFO(&data);

      if ((icm20948_->status == ICM_20948_Stat_Ok) || (icm20948_->status == ICM_20948_Stat_FIFOMoreDataAvail)) // Was valid data available?
      {
        sensor_msgs::msg::Imu imu_msg = sensor_msgs::msg::Imu();
        imu_msg.header.stamp = this->get_clock()->now();
        imu_msg.header.frame_id = frame_id_;

        if ((data.header & DMP_header_bitmap_Quat9) > 0) // We have asked for orientation data so we should receive Quat9
        {
          RCLCPP_DEBUG(this->get_logger(), "Received Quat9 data! Header: 0x%04X", data.header);
          // Q0 value is computed from this equation: Q0^2 + Q1^2 + Q2^2 + Q3^2 = 1.
          // In case of drift, the sum will not add to 1, therefore, quaternion data need to be corrected with right bias values.
          // The quaternion data is scaled by 2^30.
          // Scale to +/- 1
          double q1 = ((double)data.Quat9.Data.Q1) / (1 << 30); // Convert to double. Divide by 2^30
          double q2 = ((double)data.Quat9.Data.Q2) / (1 << 30); // Convert to double. Divide by 2^30
          double q3 = ((double)data.Quat9.Data.Q3) / (1 << 30); // Convert to double. Divide by 2^30
          double q0 = sqrt(1.0 - ((q1 * q1) + (q2 * q2) + (q3 * q3)));
          imu_msg.orientation.w = q0;
          imu_msg.orientation.x = q1;
          imu_msg.orientation.y = q2;
          imu_msg.orientation.z = q3;
          RCLCPP_DEBUG(this->get_logger(), "Quat9 data is: Q0:%f Q1:%f Q2:%f Q3:%f Accuracy:%d", q0, q1, q2, q3, data.Quat9.Data.Accuracy);
        } else {
          RCLCPP_INFO(this->get_logger(), "No orientation data; header: 0x%04X", data.header);
        }
        if (data.header & DMP_header_bitmap_Accel) {
          RCLCPP_DEBUG(this->get_logger(), "Received accelerometer data! Header: 0x%04X", data.header);
          double ax = ((double)data.Raw_Accel.Data.X) * 9.80665 * acc_fss_ / 32768.0; // Convert to m/s^2
          double ay = ((double)data.Raw_Accel.Data.Y) * 9.80665 * acc_fss_ / 32768.0; // Convert to m/s^2
          double az = ((double)data.Raw_Accel.Data.Z) * 9.80665 * acc_fss_ / 32768.0; // Convert to m/s^2
          RCLCPP_DEBUG(this->get_logger(), "Accel data is: AX:%f AY:%f AZ:%f", ax, ay, az);
          imu_msg.linear_acceleration.x = ax;
          imu_msg.linear_acceleration.y = ay;
          imu_msg.linear_acceleration.z = az;
        } else {
          RCLCPP_INFO(this->get_logger(), "No accelerometer data; header: 0x%04X", data.header);
        }
        if (data.header & DMP_header_bitmap_Gyro) {
          RCLCPP_DEBUG(this->get_logger(), "Received gyroscope data! Header: 0x%04X", data.header);
          double gx = ((double)(data.Raw_Gyro.Data.X)) * (M_PI / 180.0) * gyro_fss_ / 32768.0; // Convert to rad/s
          double gy = ((double)(data.Raw_Gyro.Data.Y)) * (M_PI / 180.0) * gyro_fss_ / 32768.0; // Convert to rad/s
          double gz = ((double)(data.Raw_Gyro.Data.Z)) * (M_PI / 180.0) * gyro_fss_ / 32768.0; // Convert to rad/s
          RCLCPP_DEBUG(this->get_logger(), "Gyro data is: GX:%f GY:%f GZ:%f", gx, gy, gz);
          imu_msg.angular_velocity.x = gx;
          imu_msg.angular_velocity.y = gy;
          imu_msg.angular_velocity.z = gz;
        } else {
          RCLCPP_INFO(this->get_logger(), "No gyroscope data; header: 0x%04X", data.header);
        }
        if (data.header & DMP_header_bitmap_Compass_Calibr) {
          RCLCPP_INFO(this->get_logger(), "Received compass data! Header: 0x%04X", data.header);
          double mx = ((double)(data.Compass_Calibr.Data.X)); // Convert to uT
          double my = ((double)(data.Compass_Calibr.Data.Y)); // Convert to uT
          double mz = ((double)(data.Compass_Calibr.Data.Z)); // Convert to uT
          RCLCPP_INFO(this->get_logger(), "Compass data is: MX:%f MY:%f MZ:%f", mx, my, mz);
        } else {
          RCLCPP_INFO(this->get_logger(), "No compass data; header: 0x%04X", data.header);
        }
        if (data.header & DMP_header_bitmap_Compass) {
          RCLCPP_INFO(this->get_logger(), "Received compass data! Header: 0x%04X", data.header);
          double mx = ((double)(data.Compass.Data.X)); // Convert to uT
          double my = ((double)(data.Compass.Data.Y)); // Convert to uT
          double mz = ((double)(data.Compass.Data.Z)); // Convert to uT
          RCLCPP_INFO(this->get_logger(), "Compass data is: MX:%f MY:%f MZ:%f", mx, my, mz);
        } else {
          RCLCPP_INFO(this->get_logger(), "No compass data; header: 0x%04X", data.header);
        }
        if (data.header & DMP_header_bitmap_Header2) {
          RCLCPP_INFO(this->get_logger(), "Received Header2; data is: 0x%04X", data.header2);
          RCLCPP_INFO(this->get_logger(), "Accuracies: Accel:%d Gyro:%d Cpass:%d", data.Accel_Accuracy, data.Gyro_Accuracy, data.Compass_Accuracy);
        }
        publisher_imu_->publish(imu_msg);
      }
    } while (icm20948_->status == ICM_20948_Stat_FIFOMoreDataAvail && rclcpp::ok());
    }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ICM20948_Node>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
