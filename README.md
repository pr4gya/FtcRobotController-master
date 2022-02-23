Vortex Robotics 14969 autonomous and teleop code for the FTC Ultimate Goal 2020-2021 season.
Teleop: Two driver controllers programmed, drivetrain and non-drivetrain. Non-drivetrain contains all non design modules not related to the wheels, as its' name suggests. Ie intake(reversed and forward movement), shooting, two servos, as well as arm and claw.
Autonomous: Several algorithms included to add precision and efficiency to our code: 
1. Angle adjustment, calculates displacement of angle using heading of inertial measurement unit gyroscope refactored as control hub is mounted on y-axis
2. deadwheel odometry coordinate positioning
3. pure pursuit localization and path tracking
4. encoders
