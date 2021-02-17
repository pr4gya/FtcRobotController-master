package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import HelperClass.OdometryRobot;


@TeleOp(name = "Odometry Positioning")
public class OdometerTest extends LinearOpMode {
    static final double threshold = 0.3;
    private ElapsedTime runtime = new ElapsedTime();
    private double rightx = 0;
    private double leftx = 0;
    private double lefty = 0;

    private OdometryRobot robot = new OdometryRobot();

    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap);
        runtime.reset();

        telemetry.addData("status", "initialized");
        telemetry.update();

        waitForStart();



        robot.resetTicks();
        //Create and start GlobalCoordinatePosition thread to constantly update the global coordinate positions
        OdometryCoordinatePosition globalPositionUpdate = new OdometryCoordinatePosition(robot.leftEncoderMotor, robot.rightEncoderMotor, robot.centerEncoderMotor);
        Thread positionThread = new Thread(globalPositionUpdate);
        positionThread.start();

        //globalPositionUpdate.reverseRightEncoder();
        //globalPositionUpdate.reverseCenterEncoder();

        while (!isStopRequested() && opModeIsActive()) {

            double move_y_axis = -gamepad1.left_stick_y;      // Remember, this is reversed!
            double move_x_axis = gamepad1.left_stick_x * 1.5; // Counteract imperfect strafing
            double pivot_turn = gamepad1.right_stick_x;

            double fl_power = move_y_axis + move_x_axis + pivot_turn;
            double bl_power = move_y_axis - move_x_axis + pivot_turn;
            double fr_power = move_y_axis - move_x_axis - pivot_turn;
            double br_power = move_y_axis + move_x_axis - pivot_turn;
            double shootPower = 0.6;

            //now we can set the powers
            robot.FLMotor.setPower(fl_power);
            robot.BLMotor.setPower(bl_power);
            robot.FRMotor.setPower(fr_power);
            robot.BRMotor.setPower(br_power);

            /*
            telemetry.addData("Left ticks", robot.getLeftTicks());
            telemetry.addData("Right ticks", robot.getRightTicks());
            telemetry.addData("Left distance", (robot.getLeftTicks() / robot.oneRotationTicks) * 2.0 * Math.PI * robot.wheelRadius);
            telemetry.addData("Right distance", (robot.getRightTicks() / robot.oneRotationTicks) * 2.0 * Math.PI * robot.wheelRadius);

            telemetry.addData("Center ticks", robot.getCenterTicks());
            telemetry.addData("X value", robot.getX());
            telemetry.addData("Y value", robot.getY());
            telemetry.addData("Theta value", robot.getTheta());
            */

            telemetry.addData("X Position", globalPositionUpdate.returnXCoordinate() / robot.countsPerInch);
            telemetry.addData("Y Position", globalPositionUpdate.returnYCoordinate() / robot.countsPerInch);
            telemetry.addData("Orientation (Degrees)", globalPositionUpdate.returnOrientation());

            telemetry.addData("Vertical left encoder position", robot.leftEncoderMotor.getCurrentPosition());
            telemetry.addData("Vertical right encoder position", robot.rightEncoderMotor.getCurrentPosition());
            telemetry.addData("horizontal encoder position", robot.centerEncoderMotor.getCurrentPosition());

            telemetry.addData("Thread Active", positionThread.isAlive());
            telemetry.update();



            telemetry.update();

//            lefty = Range.clip(gamepad1.left_stick_y, -1, 1);
//            rightx = Range.clip(gamepad1.right_stick_x, -1, 1);
//            leftx = Range.clip(gamepad1.left_stick_x, -1, 1);
//
//            if (Math.abs(gamepad1.left_stick_y) > threshold) {
//                robot.drive(-lefty, -lefty, -lefty, -lefty);
//            } else if (Math.abs(gamepad1.left_stick_x) > threshold) {
//                robot.drive(-leftx, leftx, leftx, -leftx);
//            }  else if (Math.abs(gamepad1.right_stick_x) > threshold) {
//                robot.drive(-rightx, -rightx, rightx, rightx);
//            } else {
//                robot.drive(0, 0, 0, 0);
//            }
//            idle();
        }
        robot.drive(0, 0, 0, 0);
    }
    //public double map(double x, double in_min, double in_max, double out_min, double out_max) {
    //    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    //}
}