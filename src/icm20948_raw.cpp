#include <cstdint>
#include <iostream>
#include <memory>
#include <rclcpp/logger.hpp>
#include <rclcpp/utilities.hpp>
#include "AK09916_ENUMERATIONS.h"
#include "AK09916_REGISTERS.h"
#include "ICM_20948_C.h"
#include "ICM_20948_DMP.h"
#include "ICM_20948_REGISTERS.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"
#include "ICM_20948.h"
#include "i2c_wrapper.hpp"

class ICM20948_Node : public rclcpp::Node
{
  private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_imu_;
    rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr publisher_mag_;
    void logged_call(ICM_20948_Status_e result, const std::string& action) {
      if (result != ICM_20948_Stat_Ok) {
        RCLCPP_ERROR(this->get_logger(), "Failed to %s: %d", action.c_str(), result);
        icm20948_->debugPrintStatus(result);
        icm20948_->debugPrintln("");
        throw std::runtime_error("Failed to " + action);
      } else {
        RCLCPP_DEBUG(this->get_logger(), "Successfully %s", action.c_str());
      }
    }

    void initialise_mag() {

      uint8_t whoami_bytes[2];
      mag_serif_.read(AK09916_REG_WIA1, whoami_bytes, 2, mag_serif_.user);
      if (whoami_bytes[0] != 0x48 || whoami_bytes[1] != 0x09) {
        RCLCPP_ERROR(this->get_logger(), "Failed to connect to AK09916 magnetometer: whoami result 0x%04X", whoami_bytes[0] << 8 | whoami_bytes[1]) ;
        throw std::runtime_error("Failed to connect to AK09916 magnetometer");
      } else {
        RCLCPP_INFO(this->get_logger(), "Successfully connected to AK09916 magnetometer");
      }
      uint8_t mode = AK09916_mode_cont_100hz;
      mag_serif_.write(AK09916_REG_CNTL2, &mode, 1, mag_serif_.user); // Set single measurement mode
    }

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
    std::unique_ptr<I2CWrapper> mag_i2c_wrapper_;
    ICM_20948_Serif_t mag_serif_;

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

      icm20948_->startupDefault(true);
      initialise();
      usleep(100000); // Give the sensor time to start up before we try to talk to the magnetometer
      mag_i2c_wrapper_ = std::make_unique<I2CWrapper>(i2c_bus_, MAG_AK09916_I2C_ADDR);
      mag_serif_ = mag_i2c_wrapper_->get_serif();
      initialise_mag();
      publisher_imu_ = this->create_publisher<sensor_msgs::msg::Imu>("imu", 10);
      publisher_mag_ = this->create_publisher<sensor_msgs::msg::MagneticField>("mag", 10);
      timer_ = this->create_wall_timer(std::chrono::milliseconds(5), std::bind(&ICM20948_Node::readData, this));
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
      ICM_20948_fss_t myFSS; // This uses a "Full Scale Settings" structure that can contain values for all configurable sensors
      myFSS.a = acc_fss_enum_;
      myFSS.g = gyro_fss_enum_;
      logged_call(icm20948_->setFullScale((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), myFSS), "set full scale");
      ICM_20948_dlpcfg_t dlpcfg;
      dlpcfg.a = acc_d473bw_n499bw;
      dlpcfg.g = gyr_d361bw4_n376bw5;
      logged_call(icm20948_->setDLPFcfg((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), dlpcfg), "set DLPF config");
      logged_call(icm20948_->enableDLPF(ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr, false), "enable DLPF for accelerometer");
      ICM_20948_smplrt_t mySmplrt;
      mySmplrt.g = 4; // 225Hz
      mySmplrt.a = 4; // 225Hz
      logged_call(icm20948_->setSampleRate((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), mySmplrt), "set sample rate");
      logged_call(icm20948_->enableFIFO(false), "disable FIFO");
      logged_call(icm20948_->enableDMP(false), "disable DMP");
      logged_call(icm20948_->i2cMasterEnable(false), "disable I2C master mode");
      logged_call(icm20948_->i2cMasterPassthrough(true), "enable passthrough mode");
    }



    void readData() {
    // Read any DMP data waiting in the FIFO
    // Note:
    //    readDMPdataFromFIFO will return ICM_20948_Stat_FIFONoDataAvail if no data is available.
    //    If data is available, readDMPdataFromFIFO will attempt to read _one_ frame of DMP data.
    //    readDMPdataFromFIFO will return ICM_20948_Stat_FIFOIncompleteData if a frame was present but was incomplete
    //    readDMPdataFromFIFO will return ICM_20948_Stat_Ok if a valid frame was read.
    //    readDMPdataFromFIFO will return ICM_20948_Stat_FIFOMoreDataAvail if a valid frame was read _and_ the FIFO contains more (unread) data.
    uint8_t big_endian_data[12];
    union {
      uint8_t raw[12];
      struct {
        int16_t AX;
        int16_t AY;
        int16_t AZ;
        int16_t GX;
        int16_t GY;
        int16_t GZ;  
      } Data;
    } data;
    union {
      uint8_t raw[10];
      struct {
        uint8_t dummy1;
        uint8_t status1;
        int16_t HX;
        int16_t HY;
        int16_t HZ;
        uint8_t dummy2;
        uint8_t status2;
      } Data;
    } mag_data;
    icm20948_->setBank(0); // Ensure we're in Bank 0 to read the sensor data
    icm20948_->read(AGB0_REG_ACCEL_XOUT_H, &big_endian_data[0], 12);
    mag_serif_.read(AK09916_REG_RSV2, &mag_data.raw[0], 10, mag_serif_.user);
    swab(big_endian_data, data.raw, 12); // Swap bytes to convert from big-endian to little-endian

    sensor_msgs::msg::Imu imu_msg = sensor_msgs::msg::Imu();
    imu_msg.header.stamp = this->get_clock()->now();
    imu_msg.header.frame_id = frame_id_;
    double ax = ((double)data.Data.AX) * 9.80665 * acc_fss_ / 32768.0; // Convert to m/s^2
    double ay = ((double)data.Data.AY) * 9.80665 * acc_fss_ / 32768.0; // Convert to m/s^2
    double az = ((double)data.Data.AZ) * 9.80665 * acc_fss_ / 32768.0; // Convert to m/s^2
    RCLCPP_DEBUG(this->get_logger(), "Accel data is: AX:%f AY:%f AZ:%f", ax, ay, az);
    imu_msg.linear_acceleration.x = ax;
    imu_msg.linear_acceleration.y = ay;
    imu_msg.linear_acceleration.z = az;
    double gx = ((double)(data.Data.GX)) * (M_PI / 180.0) * gyro_fss_ / 32768.0; // Convert to rad/s
    double gy = ((double)(data.Data.GY)) * (M_PI / 180.0) * gyro_fss_ / 32768.0; // Convert to rad/s
    double gz = ((double)(data.Data.GZ)) * (M_PI / 180.0) * gyro_fss_ / 32768.0; // Convert to rad/s
    RCLCPP_DEBUG(this->get_logger(), "Gyro data is: GX:%f GY:%f GZ:%f", gx, gy, gz);
    imu_msg.angular_velocity.x = gx;
    imu_msg.angular_velocity.y = gy;
    imu_msg.angular_velocity.z = gz;
    publisher_imu_->publish(imu_msg);


    if (mag_data.Data.status1 & 0x01) { // Check if magnetic data is ready
      sensor_msgs::msg::MagneticField mag_msg = sensor_msgs::msg::MagneticField();
      mag_msg.header.stamp = imu_msg.header.stamp; // Use the same timestamp as the IMU data
      mag_msg.header.frame_id = frame_id_;  
      double mx = ((double)(mag_data.Data.HX)) * 1e-6 * 32752.0/4912.0; // Convert to Tesla
      double my = ((double)(mag_data.Data.HY)) * 1e-6 * 32752.0/4912.0; // Convert to Tesla
      double mz = ((double)(mag_data.Data.HZ)) * 1e-6 * 32752.0/4912.0; // Convert to Tesla
      mag_msg.magnetic_field.x = mx;
      mag_msg.magnetic_field.y = my;
      mag_msg.magnetic_field.z = mz;
      RCLCPP_DEBUG(this->get_logger(), "Magnetic data is: MX:%f MY:%f MZ:%f", mx, my, mz);
      publisher_mag_->publish(mag_msg);
    }
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
