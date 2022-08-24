# ROS2 Motor Driver Package that talks to the Pololu Maestro Mini

This ROS2 package essentially talks to the on board Pololu Maestro Mini Servo Controller that is used to controller the steering motor and the drive motor for the Autonomous 1/10th Car. Currently the Node takes in data in the form of throttle effort and steering angle, between -45 and 45 degrees.

## Changing the controller values

The Pololu controller takes in values in quarter-microseconds. Basically for our case, both the servo and the drive motor takes in values from 3000 quarter-us to 9000 quarter-us, with a neutral point at 6000 quarter-microseconds. There is a class inside the [`motor_driver/motor_controller.py` file, that essentially sends serial communications between the ROS2 motor_controller subscriber node to the hardware. **BEWARE** Changing some of the values inside that class may break the ROS2 Node, change with caution.

The speed on the drive motor is currently limited to 1/3rd of it's max rated speed for safety, and the acceleration and speed to target is currently unrestricted, these can be changed to suit desired needs.

## How Radius of Curvature and Yaw Rate is Calculated

The motor_controller Node is essentially taking twist messaging and converting it back into a format it can understand, a steering angle and throttle effort. The twist consists of `Twist.linear.x` or forward velocity and `Twist.angular.z` which is the yaw rate. The yaw rate is converted into a radius of curvature and then back into steering angle. The formulas for it are as follows:
$$ R = {L \over tan(\delta)}$$
where $R$ is the radius of curvature, $L$ is the wheelbase which is .313 for our cars and $\delta$ is the steering angle. From here to calculate yaw rate the formula is as follows 
$$ \dot\phi = {\dot x \over R}$$
where $\dot\phi$ is the yaw rate and $\dot x$ is the linear velocity. 
