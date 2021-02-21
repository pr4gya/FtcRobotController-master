package Odometry.Main;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;

import HelperClass.OdometryRobot;


/**
 * Created by Sarthak on 6/1/2019.
 * Odometry system calibration. Run this OpMode to generate the necessary constants to calculate the robot's global position on the field.
 * The Global Positioning Algorithm will not function and will throw an error if this program is not run first
 */
@TeleOp(name = "Odometry System Calibration", group = "Calibration")
public class OdometryCalibration extends LinearOpMode {
    public Orientation angles;
    final double PIVOT_SPEED = 0.5;

    // The amount of encoder ticks for each inch the robot moves. THIS WILL CHANGE FOR EACH ROBOT AND NEEDS TO BE UPDATED HERE
    // Using Rev Bore Encoder with 8192 Counts per revolution https://www.revrobotics.com/rev-11-1271/
    // Deadwheel Diamater of 2 Inch
    // Hence Counter_per_inch = 8192 / ( 2* Pi * radius )
    final double COUNTS_PER_INCH = 1304.45;

    // Using Rev Bore Encoder with 8192 Counts per revolution https://www.revrobotics.com/rev-11-1271/
    //
    ElapsedTime timer = new ElapsedTime();

    double horizontalTickOffset = 0;


    private OdometryRobot robot = new OdometryRobot();

    //Text files to write the values to. The files are stored in the robot controller under Internal Storage\FIRST\settings
    File wheelBaseSeparationFile = AppUtil.getInstance().getSettingsFile("wheelBaseSeparation.txt");
    File horizontalTickOffsetFile = AppUtil.getInstance().getSettingsFile("horizontalTickOffset.txt");

    @Override
    public void runOpMode() throws InterruptedException {
         robot.init(hardwareMap);

        telemetry.addData("Odometry System Calibration Status", "IMU Init Complete");
        telemetry.clear();

        //Odometry System Calibration Init Complete
        telemetry.addData("IMU Angle", getHeadingAngle());
        telemetry.addData("Left Encoder Position", -robot.leftEncoderMotor.getCurrentPosition());
        telemetry.addData("Right Encoder Position", robot.rightEncoderMotor.getCurrentPosition());
        telemetry.addData("Center Encoder Position", robot.centerEncoderMotor.getCurrentPosition());
        telemetry.addData("Odometry System Calibration Status", "Init Complete");
        telemetry.update();
        //robot.resetTicks();

        waitForStart();

        //Begin calibration (if robot is unable to pivot at these speeds, please adjust the constant at the top of the code
        while(Math.abs(getHeadingAngle()) < 90 && opModeIsActive()){
            robot.FRMotor.setPower(-PIVOT_SPEED);
            robot.BRMotor.setPower(-PIVOT_SPEED);
            robot.FLMotor.setPower(PIVOT_SPEED);
            robot.BLMotor.setPower(PIVOT_SPEED);
            if(Math.abs(getHeadingAngle()) < 60) {
                setPowerAll(-PIVOT_SPEED, -PIVOT_SPEED, PIVOT_SPEED, PIVOT_SPEED);
            }else{
                setPowerAll(-PIVOT_SPEED/2, -PIVOT_SPEED/2, PIVOT_SPEED/2, PIVOT_SPEED/2);
            }

            telemetry.addData("IMU Angle", getHeadingAngle());
            telemetry.update();
        }

        //Stop the robot
        setPowerAll(0, 0, 0, 0);
        timer.reset();
        while(timer.milliseconds() < 1000 && opModeIsActive()){
            telemetry.addData("IMU Angle", getHeadingAngle());
            telemetry.update();
        }

        //Record IMU and encoder values to calculate the constants for the global position algorithm
        double angle = getHeadingAngle();

        /*
        Encoder Difference is calculated by the formula (leftEncoder - rightEncoder)
        Since the left encoder is also mapped to a drive motor, the encoder value needs to be reversed with the negative sign in front
        THIS MAY NEED TO BE CHANGED FOR EACH ROBOT
       */
        double encoderDifference = Math.abs(robot.leftEncoderMotor.getCurrentPosition()) + (Math.abs(robot.rightEncoderMotor.getCurrentPosition()));

        double verticalEncoderTickOffsetPerDegree = encoderDifference/angle;

        double wheelBaseSeparation = (2*90*verticalEncoderTickOffsetPerDegree)/(Math.PI*COUNTS_PER_INCH);

        horizontalTickOffset = robot.centerEncoderMotor.getCurrentPosition() / Math.toRadians(getHeadingAngle());

        //Write the constants to text files
        ReadWriteFile.writeFile(wheelBaseSeparationFile, String.valueOf(wheelBaseSeparation));
        ReadWriteFile.writeFile(horizontalTickOffsetFile, String.valueOf(horizontalTickOffset));

        while(opModeIsActive()){
            telemetry.addData("Odometry System Calibration Status", "Calibration Complete");
            //Display calculated constants
            telemetry.addData("Wheel Base Separation", wheelBaseSeparation);    //15.2
            telemetry.addData("Horizontal Encoder Offset", horizontalTickOffset); //130

            //Display raw values
            telemetry.addData("IMU Angle", getHeadingAngle());
            telemetry.addData("Vertical Left Position", -robot.leftEncoderMotor.getCurrentPosition());
            telemetry.addData("Vertical Right Position", robot.rightEncoderMotor.getCurrentPosition());
            telemetry.addData("Horizontal Position", robot.centerEncoderMotor.getCurrentPosition());
            telemetry.addData("Vertical Encoder Offset", verticalEncoderTickOffsetPerDegree);

            //Update values
            telemetry.update();
        }
    }


    /**
     * Gets the orientation of the robot using the REV IMU
     * @return the angle of the robot
     */
    private double getHeadingAngle(){
        angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        if (angles.firstAngle < 0) angles.firstAngle = angles.firstAngle * (-1.0f);
        //else angles.firstAngle = -angles.firstAngle + 360.0f;
        return (angles.firstAngle);
    }

    /**
     * Sets power to all four drive motors
     * @param rf power for right front motor
     * @param rb power for right back motor
     * @param lf power for left front motor
     * @param lb power for left back motor
     */
    private void setPowerAll(double rf, double rb, double lf, double lb){
        robot.FRMotor.setPower(rf);
        robot.BRMotor.setPower(rb);
        robot.FLMotor.setPower(lf);
        robot.BLMotor.setPower(lb);
    }

}