TransducerM IMU Driver Setup:

1. Copy the ROS package

Copy the tm_imu folder into:

~/sensor_drivers_ws/src/


2. Copy the udev rules

Copy:

99-transducerm.rules

to:

/etc/udev/rules.d/


3. Reload the udev rules

sudo udevadm control --reload-rules
sudo udevadm trigger


4. Plug in all TransducerM IMUs

Check that the device links were created:

ls -l /dev/tm_imu_*

You should see:

/dev/tm_imu_1
/dev/tm_imu_2
/dev/tm_imu_3
/dev/tm_imu_4


5. Give the user serial-port permission

sudo usermod -aG dialout $USER

(Log out and log back in, or restart the computer)

Check:

groups

Make sure "dialout" appears in the list.


6. Install dependencies

cd ~/sensor_drivers_ws

rosdep update

rosdep install --from-paths src/tm_imu --ignore-src -r -y


7. Build only the tm_imu package

cd ~/sensor_drivers_ws

colcon build --packages-select tm_imu --symlink-install


8. Source the workspace

source ~/sensor_drivers_ws/install/setup.bash

(add this line to ~/.bashrc so it is sourced automatically in new terminals)


9. Check that the package is detected

cd ~/sensor_drivers_ws

colcon list

You should see:

tm_imu    src/tm_imu


10. Launch all TransducerM IMUs

ros2 launch tm_imu multi_tm_imu.launch.py


11. Check the topics

ros2 topic list | grep tm_imu

You should see:

/tm_imu_1/imu_data
/tm_imu_1/imu_data_mag
/tm_imu_1/imu_data_rpy

/tm_imu_2/imu_data
/tm_imu_2/imu_data_mag
/tm_imu_2/imu_data_rpy

/tm_imu_3/imu_data
/tm_imu_3/imu_data_mag
/tm_imu_3/imu_data_rpy

/tm_imu_4/imu_data
/tm_imu_4/imu_data_mag
/tm_imu_4/imu_data_rpy


12. Test the IMU data rate

ros2 topic hz /tm_imu_1/imu_data

ros2 topic hz /tm_imu_2/imu_data

ros2 topic hz /tm_imu_3/imu_data

ros2 topic hz /tm_imu_4/imu_data
