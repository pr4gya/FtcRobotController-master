package Odometry.Main;


import HelperClass.Robot;
import Odometry.OdoRobot;
import Odometry.geometry.Pose2d;
import Odometry.geometry.Rotation2d;

public class SwerveDriveOdometry {


    OdoRobot Odorobot = new OdoRobot();
    Robot robot = new Robot();

    Pose2d m_pose = new Pose2d();

    SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(
            robot.getRobotAngle(), new Pose2d(), new Rotation2d());


    public SwerveDriveOdometry(float robotAngle, Pose2d pose2d, Rotation2d rotation2d) {





        public void Periodic() {
// Get my gyro angle. We are negating the value because gyros return positive
// values as the robot turns clockwise.
          float gyroAngle = Rotation2d.fromDegrees(-robot.getRobotAngle());

// Update the pose
            m_pose = m_odometry.update(gyroAngle, m_leftEncoder.getDistance(), m_rightEncoder.getDistance());

    }
}

