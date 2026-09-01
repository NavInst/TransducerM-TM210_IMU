#include "tm_imu/tm_imu_node.hpp"

EasyObjectDictionary eOD;
EasyProfile eP(&eOD);

#ifdef DEBUG_MODE
#define DEBUG_MODE_PRINT_TIMER_MS_  (10000)  // 10 seconds
#endif

TMSerial::TMSerial() : rclcpp::Node("tm_imu")
{
    // Declare node's parameters default value 
    // [NOT NECESSARY TO CHANGE THE FOLLOWING, USE ../config/params.yaml INSTEAD !]
    this->declare_parameter("imu_baudrate",    115200);
    this->declare_parameter("imu_port",        "/dev/ttyUSB0");
    this->declare_parameter("imu_frame_id",    "imu");
    this->declare_parameter("parent_frame_id", "base_link");
    this->declare_parameter("timer_period",     50);            // Unit: ms
    this->declare_parameter("transform",        std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
    // Declare a serial object
    serialib1 = new serialib;
    SerialportOpen();
    // Performance monitor
    #ifdef DEBUG_MODE
    count    = 0;
    count2   = 0;
    timer_10 = this->create_wall_timer(std::chrono::milliseconds(DEBUG_MODE_PRINT_TIMER_MS_), std::bind(&TMSerial::TimerCallback2, this));
    #endif
    // Set frame id
    imu_data_msg.header.frame_id     = this->get_parameter("imu_frame_id").as_string();
    imu_data_rpy_msg.header.frame_id = this->get_parameter("imu_frame_id").as_string();
    imu_data_mag_msg.header.frame_id = this->get_parameter("imu_frame_id").as_string();
    imu_temperature_msg.header.frame_id = this->get_parameter("imu_frame_id").as_string();
    imu_sensor_time_msg.header.frame_id = this->get_parameter("imu_frame_id").as_string();
    imu_sensor_time_msg.source = "TM210 sensor timestamp in microseconds since power-on (rollover-corrected)";
    // Create publisher
    publisher_IMU     = this->create_publisher<sensor_msgs::msg::Imu>("imu_data", 10);
    publisher_IMU_RPY = this->create_publisher<sensor_msgs::msg::MagneticField>("imu_data_rpy", 10);
    publisher_IMU_MAG = this->create_publisher<sensor_msgs::msg::MagneticField>("imu_data_mag", 10);
    publisher_IMU_TEMP = this->create_publisher<sensor_msgs::msg::Temperature>("temperature", 10);
    publisher_IMU_SENSOR_TIME = this->create_publisher<sensor_msgs::msg::TimeReference>("sensor_time", 10);
    // Create timer
    std::chrono::milliseconds period = std::chrono::milliseconds(this->get_parameter("timer_period").as_int());
    timer_ = this->create_wall_timer(period, std::bind(&TMSerial::TimerCallback, this));

    // Request the TM210 STATUS packet once per second. STATUS contains
    // temperature and the sensor-reported internal update rate.
    status_timer_ = this->create_wall_timer(
        std::chrono::seconds(1),
        std::bind(&TMSerial::StatusRequestCallback, this));

    // Create tf_broadcaster
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
}


TMSerial::~TMSerial()
{
    if(serialib1){
        serialib1->closeDevice();
        delete serialib1;
        serialib1 = 0;
    }
}


void TMSerial::TimerCallback()
{
    // These flags indicate whether this particular read produced fresh
    // sensor-time or temperature data.
    sensor_time_updated_ = false;
    temperature_updated_ = false;

    // Read values from the IMU sensor.
    bool res = OnSerialRX();
    if (!res) return;

    // Keep the existing ROS/computer timestamp behavior.
    const auto ros_stamp = this->get_clock()->now();
    imu_data_msg.header.stamp        = ros_stamp;
    imu_data_rpy_msg.header.stamp    = ros_stamp;
    imu_data_mag_msg.header.stamp    = ros_stamp;
    imu_temperature_msg.header.stamp = ros_stamp;
    imu_sensor_time_msg.header.stamp = ros_stamp;

    FillCovarianceMatrices();

    // Existing topics remain unchanged.
    PublishTransform();
    publisher_IMU->publish(imu_data_msg);
    publisher_IMU_RPY->publish(imu_data_rpy_msg);
    publisher_IMU_MAG->publish(imu_data_mag_msg);

    // Sensor time is taken from the TM210 data packet itself.
    if (sensor_time_updated_) {
        publisher_IMU_SENSOR_TIME->publish(imu_sensor_time_msg);
    }

    // Temperature is published when a STATUS (or COMBO) packet is received.
    if (temperature_updated_) {
        publisher_IMU_TEMP->publish(imu_temperature_msg);
    }
}


#ifdef DEBUG_MODE
void TMSerial::TimerCallback2()
{
    RCLCPP_INFO(this->get_logger(), 
    "Rx byte cnt=%d, TrasnducerM pkg cnt = %d (%f Hz)", count2, count, 1000*((float)count)/(DEBUG_MODE_PRINT_TIMER_MS_));
    count = 0;
    count2 = 0;
}
#endif


void TMSerial::StatusRequestCallback()
{
    // We learn the device's EasyProfile Short-ID from its incoming packets.
    // Until at least one packet has arrived, there is no destination ID to use.
    if (!sensor_id_known_ || serialib1 == nullptr || !serialib1->isDeviceOpen()) {
        return;
    }

    // Build an EasyProfile REQUEST asking this TM210 for EP_CMD_STATUS_.
    if (eOD.Write_Ep_Request(sensor_short_id_, EP_CMD_STATUS_) != EP_SUCC_) {
        return;
    }

    EP_ID_TYPE_ to_id = EP_ID_UNSPECIFIED_;
    char* package_data = nullptr;
    int package_size = 0;

    if (eP.On_SendPkg(EP_CMD_REQUEST_, &to_id, &package_data, &package_size) != EP_SUCC_) {
        return;
    }

    if (package_data == nullptr || package_size <= 0) {
        return;
    }

    // Send the request over the already-open serial connection.
    // The TM210's STATUS response will be decoded in OnSerialRX().
    serialib1->writeBytes(package_data, static_cast<unsigned int>(package_size));
}


char TMSerial::SerialportOpen()
{
    int Ret;
    unsigned int baudrate = this->get_parameter("imu_baudrate").as_int();
    Ret=serialib1->openDevice(this->get_parameter("imu_port").as_string().c_str(), baudrate,
        SERIAL_DATABITS_8, SERIAL_PARITY_NONE, SERIAL_STOPBITS_1);                         // Open serial link at the specified baudrate
    if (Ret!=1) {                                                                          // If an error occured...
        RCLCPP_INFO(this->get_logger(), "Error while opening port. Permission problem ?"); // ... display a message ...
        RCLCPP_INFO(this->get_logger(), "imu_port:%s imu_baudrate:%d",this->get_parameter("imu_port").as_string().c_str(),baudrate);
        return Ret;                                                                        // ... quit the application
    }
    RCLCPP_INFO(this->get_logger(), "Serial port opened successfully !");
    RCLCPP_INFO(this->get_logger(), "imu_port:%s imu_baudrate:%d",this->get_parameter("imu_port").as_string().c_str(),baudrate);
    return 1;
}


bool TMSerial::OnSerialRX()
{
    char serialBuffer[1024];
    int ret = serialib1->readBytes(serialBuffer, sizeof(serialBuffer),1,100);
    #ifdef DEBUG_MODE
    //RCLCPP_INFO(this->get_logger(), "rxsize = %d",ret);
    count2 += ret;
    #endif
    if (ret <= 0) return false;
                                                            // Step 1: read the received data buffer of the Serial Port
    char*  rxData = serialBuffer;                           //         and then convert it to data types acceptable by the
    int    rxSize = ret;                                    //         Communication Abstraction Layer (CAL).
    Ep_Header header;
    while(EP_SUCC_ == eP.On_RecvPkg(rxData, rxSize, &header)){ // Step 2: Tell the CAL that new data has arrived.
                                                            //         It does not matter if the new data only contains a fraction
                                                            //         of a complete package, nor does it matter if the data is broken
                                                            //         during the transmission. On_RecvPkg() will only return EP_SUCC_
                                                            //         when a complete and correct package has arrived.
        rxData = 0;                                         //         The while loop ensures no package is omitted. 
    	rxSize = 0;
    	
        // Example Reading of the Short ID of the device who send the data:
        uint32 fromId = header.fromId;                      // Step 3.1:  Now we are able to read the received payload data.
                                                            //            header.fromId tells us from which Motion Module the data comes.
                                                            
        //Supress "parameter unused" complier warning:
        (void)fromId;
        
        switch(header.cmd){                                 // Step 3.2: header.cmd tells what kind of data is inside the payload.
            case EP_CMD_ACK_:{                              //           We can use a switch() as demonstrated here to do different
                Ep_Ack ep_Ack;                              //           tasks for different types of data.
                if(EP_SUCC_ == eOD.Read_Ep_Ack(&ep_Ack)){}
            }break;
            case EP_CMD_STATUS_:{
                Ep_Status ep_Status;
                if(EP_SUCC_ == eOD.Read_Ep_Status(&ep_Status)){
                    // STATUS provides a float temperature in degrees Celsius.
                    imu_temperature_msg.temperature =
                        static_cast<double>(ep_Status.temperature);
                    imu_temperature_msg.variance = 0.0;
                    temperature_updated_ = true;

                    if (!status_seen_logged_) {
                        RCLCPP_INFO(
                            this->get_logger(),
                            "Receiving EP_CMD_STATUS_: temperature=%.2f C, sensor update rate=%u Hz",
                            static_cast<double>(ep_Status.temperature),
                            static_cast<unsigned int>(ep_Status.updateRate)
                        );
                        status_seen_logged_ = true;
                    }
                }
            }break;
            case EP_CMD_Raw_GYRO_ACC_MAG_:{
                Ep_Raw_GyroAccMag ep_Raw_GyroAccMag;
                if(EP_SUCC_ == eOD.Read_Ep_Raw_GyroAccMag(&ep_Raw_GyroAccMag)){
                    // This timestamp is generated by the TM210 when the raw
                    // gyro/accelerometer/magnetometer sample is taken.
                    UpdateSensorTimestamp(ep_Raw_GyroAccMag.timeStamp);
                    sensor_time_updated_ = true;
                    
                    
                    float gyro_x = ep_Raw_GyroAccMag.gyro[0];
                    float gyro_y = ep_Raw_GyroAccMag.gyro[1];
                    float gyro_z = ep_Raw_GyroAccMag.gyro[2];
                    
                    constexpr float TM210_GRAVITY = 9.794f;
                    float acc_x = ep_Raw_GyroAccMag.acc[0] * TM210_GRAVITY;
                    float acc_y = ep_Raw_GyroAccMag.acc[1] * TM210_GRAVITY;
                    float acc_z = ep_Raw_GyroAccMag.acc[2] * TM210_GRAVITY;

                    float mag_x = ep_Raw_GyroAccMag.mag[0];
                    float mag_y = ep_Raw_GyroAccMag.mag[1];
                    float mag_z = ep_Raw_GyroAccMag.mag[2];

                    imu_data_msg.angular_velocity.x =  gyro_x;
                    imu_data_msg.angular_velocity.y =  gyro_y;
                    imu_data_msg.angular_velocity.z =  gyro_z;

                    imu_data_msg.linear_acceleration.x = acc_x; 
                    imu_data_msg.linear_acceleration.y = acc_y;
                    imu_data_msg.linear_acceleration.z = acc_z;

                    imu_data_mag_msg.magnetic_field.x = mag_x;
                    imu_data_mag_msg.magnetic_field.y = mag_y;
                    imu_data_mag_msg.magnetic_field.z = mag_z;
                    #ifdef DEBUG_MODE
                    //RCLCPP_INFO(this->get_logger(), "RAW pkg");
                    #endif
                }
            }break;
            case EP_CMD_Q_S1_S_:{
                Ep_Q_s1_s ep_Q_s1_s;
                if(EP_SUCC_ == eOD.Read_Ep_Q_s1_s(&ep_Q_s1_s)){}
            }break;
            case EP_CMD_Q_S1_E_:{
                Ep_Q_s1_e ep_Q_s1_e;
                if(EP_SUCC_ == eOD.Read_Ep_Q_s1_e(&ep_Q_s1_e)){ // Step 3.3: If we decided that the received Quaternion should be used,
                                                                //           Here is an example of how to access the Quaternion data.
                    float q1 = ep_Q_s1_e.q[0];
                    float q2 = ep_Q_s1_e.q[1];
                    float q3 = ep_Q_s1_e.q[2];
                    float q4 = ep_Q_s1_e.q[3];
                    // uint32 timeStamp = ep_Q_s1_e.timeStamp;  //TimeStamp indicates the time point (since the Module has been powered on),
                                                                //when this particular set of Quaternion was calculated. (Unit: uS)
                                                                //Note that overflow will occure when the uint32 type reaches its maximum value.
                                                                //The ID indicates the device Short ID telling which Motion Module the data comes from.
                    // uint32 deviceId  = ep_Q_s1_e.header.fromId;
                    imu_data_msg.orientation.w = q1;
                    imu_data_msg.orientation.x = q2;
                    imu_data_msg.orientation.y = q3;
                    imu_data_msg.orientation.z = q4;
                    #ifdef DEBUG_MODE
                    //RCLCPP_INFO(this->get_logger(), "QS1 pkg");
                    #endif
                }
            }break;
            case EP_CMD_EULER_S1_S_:{
                Ep_Euler_s1_s ep_Euler_s1_s;
                if(EP_SUCC_ == eOD.Read_Ep_Euler_s1_s(&ep_Euler_s1_s)){}
            }break;
            case EP_CMD_EULER_S1_E_:{
                Ep_Euler_s1_e ep_Euler_s1_e;
                if(EP_SUCC_ == eOD.Read_Ep_Euler_s1_e(&ep_Euler_s1_e)){}
            }break;
            case EP_CMD_RPY_:{
                Ep_RPY ep_RPY;
                if(EP_SUCC_ == eOD.Read_Ep_RPY(&ep_RPY)){        //Another Example reading of the received Roll Pitch and Yaw
                    float roll = ep_RPY.roll;
                    float pitch = ep_RPY.pitch;
                    float yaw = ep_RPY.yaw;
                    //uint32 timeStamp = ep_RPY.timeStamp;
                    //uint32 deviceId  = ep_RPY.header.fromId;
                    imu_data_rpy_msg.magnetic_field.x = roll;
                    imu_data_rpy_msg.magnetic_field.y = pitch;
                    imu_data_rpy_msg.magnetic_field.z = yaw;
                    #ifdef DEBUG_MODE
                    //RCLCPP_INFO(this->get_logger(), "RPY pkg");
                    #endif
                }
            }break;
            case EP_CMD_GRAVITY_:{
                Ep_Gravity ep_Gravity;
                if(EP_SUCC_ == eOD.Read_Ep_Gravity(&ep_Gravity)){}
            }break;
            case EP_CMD_COMBO_: {
                Ep_Combo ep_Combo;
                if(EP_SUCC_ == eOD.Read_Ep_Combo(&ep_Combo)){
                    // EP_CMD_COMBO_ already contains both the TM210 sensor timestamp
                    // and temperature. The timestamp unit is microseconds since the
                    // sensor was powered on; temperature is integer degrees Celsius.
                    imu_temperature_msg.temperature = static_cast<double>(ep_Combo.temperature);
                    // sensor_msgs/Temperature defines variance=0 as unknown.
                    imu_temperature_msg.variance = 0.0;
                    UpdateSensorTimestamp(ep_Combo.timeStamp);
                    sensor_time_updated_ = true;
                    temperature_updated_ = true;

                    // Print this once so it is easy to verify that the IMU is actually
                    // streaming Combo packets and to see the rate reported by the sensor.
                    if (!combo_seen_logged_) {
                        RCLCPP_INFO(
                            this->get_logger(),
                            "Receiving EP_CMD_COMBO_ packets; sensor reports update rate: %u Hz",
                            static_cast<unsigned int>(ep_Combo.updateRate) * 10U
                        );
                        combo_seen_logged_ = true;
                    }

                    // Accelerometer:
                    constexpr float TM210_GRAVITY = 9.794f;
                    float ax = (ep_Combo.ax)*(1e-5f)* TM210_GRAVITY;                     // Unit: 1g, 1g = 9.794m/(s^2)
                    float ay = (ep_Combo.ay)*(1e-5f)* TM210_GRAVITY;
                    float az = (ep_Combo.az)*(1e-5f)* TM210_GRAVITY;
                    // Gyroscope:
                    float wx = (ep_Combo.wx)*(1e-5f);                     // Unit: rad/s
                    float wy = (ep_Combo.wy)*(1e-5f);
                    float wz = (ep_Combo.wz)*(1e-5f);
                    // Magnetometer:
                    float mx = (ep_Combo.mx)*(1e-3f);                     // Unit: one earth magnetic field
                    float my = (ep_Combo.my)*(1e-3f);                     // vector (mx, my, mz) is used as direction reference of the local magnetic field.
                    float mz = (ep_Combo.mz)*(1e-3f);
                    // Quaternion in (q1,q2,q3,q4)=(w,x,y,z) format
                    float q1 = (ep_Combo.q1)*(1e-7f);
                    float q2 = (ep_Combo.q2)*(1e-7f);
                    float q3 = (ep_Combo.q3)*(1e-7f);
                    float q4 = (ep_Combo.q4)*(1e-7f);
                    // RPY:
                    float roll  = (ep_Combo.roll)*(1e-2f);                // Unit: degree
                    float pitch = (ep_Combo.pitch)*(1e-2f);               // Unit: degree
                    float yaw   = (ep_Combo.yaw)*(1e-2f);                 // Unit: degree

                    imu_data_msg.angular_velocity.x =  wx;
                    imu_data_msg.angular_velocity.y =  wy;
                    imu_data_msg.angular_velocity.z =  wz;

                    imu_data_msg.linear_acceleration.x = ax; 
                    imu_data_msg.linear_acceleration.y = ay;
                    imu_data_msg.linear_acceleration.z = az;

                    imu_data_msg.orientation.x = q1;
                    imu_data_msg.orientation.y = q2;
                    imu_data_msg.orientation.z = q3;
                    imu_data_msg.orientation.w = q4;

                    imu_data_rpy_msg.magnetic_field.x = roll;
                    imu_data_rpy_msg.magnetic_field.y = pitch;
                    imu_data_rpy_msg.magnetic_field.z = yaw;

                    imu_data_mag_msg.magnetic_field.x = mx;
                    imu_data_mag_msg.magnetic_field.y = my;
                    imu_data_mag_msg.magnetic_field.z = mz;

                    #ifdef DEBUG_MODE
                    //RCLCPP_INFO(this->get_logger(), "Combo pkg");
                    #endif
                    }
                } break;
            }
        #ifdef DEBUG_MODE
        count++;
        #endif
    } // while()
    return true;
}



void TMSerial::UpdateSensorTimestamp(uint32_t raw_timestamp_us)
{
    // The TM210 timestamp is uint32 microseconds since power-on. A uint32
    // timestamp wraps every ~71.58 minutes, so extend it to uint64 here.
    if (sensor_timestamp_initialized_ && raw_timestamp_us < last_sensor_timestamp_raw_) {
        const uint32_t backward_jump = last_sensor_timestamp_raw_ - raw_timestamp_us;

        if (backward_jump > 0x80000000U) {
            // Normal uint32 rollover from a value near UINT32_MAX to a small value.
            ++sensor_timestamp_wrap_count_;
        } else {
            // A smaller backward jump is more consistent with a sensor reboot/reset.
            sensor_timestamp_wrap_count_ = 0;
        }
    }

    last_sensor_timestamp_raw_ = raw_timestamp_us;
    sensor_timestamp_initialized_ = true;

    sensor_timestamp_us_ =
        (sensor_timestamp_wrap_count_ << 32) | static_cast<uint64_t>(raw_timestamp_us);

    // Store the sensor's relative time in sensor_msgs/TimeReference.
    // header.stamp remains ROS/computer time; time_ref is TM210 time.
    imu_sensor_time_msg.time_ref.sec =
        static_cast<int32_t>(sensor_timestamp_us_ / 1000000ULL);
    imu_sensor_time_msg.time_ref.nanosec =
        static_cast<uint32_t>((sensor_timestamp_us_ % 1000000ULL) * 1000ULL);
}

void TMSerial::FillCovarianceMatrices()
{
    for(int i = 0; i < 9; i++){
        imu_data_msg.orientation_covariance[i]        = 0.1;
        imu_data_msg.angular_velocity_covariance[i]   = 0.1;
        imu_data_msg.linear_acceleration_covariance[i]= 0.1;
        imu_data_rpy_msg.magnetic_field_covariance[i] = 0.1;
        imu_data_mag_msg.magnetic_field_covariance[i] = 0.1;
    }
}


void TMSerial::PublishTransform()
{
    geometry_msgs::msg::TransformStamped transform_;
    transform_.header.stamp = this->get_clock()->now();
    transform_.header.frame_id = "world";
    transform_.child_frame_id = this->get_parameter("imu_frame_id").as_string();
    transform_.transform.translation.x = this->get_parameter("transform").as_double_array()[0];
    transform_.transform.translation.y = this->get_parameter("transform").as_double_array()[1];
    transform_.transform.translation.z = this->get_parameter("transform").as_double_array()[2];

    transform_.transform.rotation.x = imu_data_msg.orientation.x;
    transform_.transform.rotation.y = imu_data_msg.orientation.y;
    transform_.transform.rotation.z = imu_data_msg.orientation.z;
    transform_.transform.rotation.w = imu_data_msg.orientation.w;
    tf_broadcaster_->sendTransform(transform_);
}


int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TMSerial>());
    rclcpp::shutdown();
    return 0;
}
