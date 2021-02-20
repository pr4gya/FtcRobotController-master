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

    double distanceToTargetX = 0;
    double distanceToTargetY = 0;
    double distance = 0;
    double targetX = 0;
    double targetY = 24;
    int gotoTarget = 1;
    boolean move_fwd = false;


    OdometryRobot robot = new OdometryRobot();

    OdometryCoordinatePosition globalPositionUpdate;


    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap);
        runtime.reset();

        telemetry.addData("status", "initialized");
        telemetry.update();

        waitForStart();


        //Create and start GlobalCoordinatePosition thread to constantly update the global coordinate positions
        OdometryCoordinatePosition globalPositionUpdate = new OdometryCoordinatePosition(robot.leftEncoderMotor, robot.rightEncoderMotor, robot.centerEncoderMotor);
        Thread positionThread = new Thread(globalPositionUpdate);
        positionThread.start();

        //gotoPosition(0 * robot.countsPerInch, 24 * robot.countsPerInch, 0.5, 0 , 1.0 * robot.countsPerInch);

        globalPositionUpdate.reverseRightEncoder();
        globalPositionUpdate.reverseLeftEncoder();

        //globalPositionUpdate.reverseCenterEncoder();


         distanceToTargetX = targetX * robot.countsPerInch - globalPositionUpdate.returnXCoordinate();
         distanceToTargetY = targetY * robot.countsPerInch - globalPositionUpdate.returnYCoordinate();
         distance = Math.hypot(distanceToTargetX, distanceToTargetY);

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

            if (gamepad1.right_trigger > 0 ) {
                move_fwd = true;

            }

            if (distanceToTargetY >= 0 && move_fwd == true ) {
                gotoTarget = 0;
                double robotMovementAngle = Math.toDegrees(Math.atan2(distanceToTargetX, distanceToTargetY));
                //double[] wheelPwr = rawSlide(0.0, 0.2, 0.0, 1.0);
                robot.FLMotor.setPower(0.3);
                robot.BLMotor.setPower(0.3);
                robot.FRMotor.setPower(0.3);
                robot.BRMotor.setPower(0.3);

                distanceToTargetX = targetX * robot.countsPerInch - globalPositionUpdate.returnXCoordinate();
                distanceToTargetY = targetY * robot.countsPerInch - globalPositionUpdate.returnYCoordinate();
                distance = Math.hypot(distanceToTargetX, distanceToTargetY);
            }


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


            telemetry.addData("Distance ", distance / robot.countsPerInch);

            telemetry.addData("distanceToTargetX ", distanceToTargetX / robot.countsPerInch);
            telemetry.addData("distanceToTargetY ", distanceToTargetY / robot.countsPerInch);

            telemetry.addData("X Position", globalPositionUpdate.returnXCoordinate() / robot.countsPerInch);
            telemetry.addData("Y Position", globalPositionUpdate.returnYCoordinate() / robot.countsPerInch);
            telemetry.addData("Odometry Angle (Degree)", globalPositionUpdate.returnAngleDegree());
            telemetry.addData("IMU Robot Angle: ", "%f", robot.imuRobotAngle());
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
        //robot.drive(0, 0, 0, 0);
        //Stop the thread
        globalPositionUpdate.stop();
    }
    //public double map(double x, double in_min, double in_max, double out_min, double out_max) {
    //    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    //}



    public void gotoPosition(double targetX, double targetY, double robotPwr, double desiredOrientation, double allowedDistanceError) {

//        double distanceToTargetX = targetX - globalPositionUpdate.returnXCoordinate();
//        double distanceToTargetY = targetY - globalPositionUpdate.returnYCoordinate();

//        double distance = Math.hypot(distanceToTargetX, distanceToTargetY);
//
//
//        while (opModeIsActive() && distance > allowedDistanceError) {
//
//             distanceToTargetX = targetX - globalPositionUpdate.returnXCoordinate();
//             distanceToTargetY = targetY - globalPositionUpdate.returnYCoordinate();
//
//            double robotMovementAngle = Math.toDegrees(Math.atan2(distanceToTargetX, distanceToTargetY));
//
//
//            double robot_movement_x_direction = calculateX(robotMovementAngle, robotPwr);
//            double robot_movement_y_direction = calculateY(robotMovementAngle, robotPwr);
//            double pivotCorrection = desiredOrientation - globalPositionUpdate.returnAngle();
//
//            double[] wheelPwr = rawSlide(0.0, 0.5, 0.0, 1.0);
//            //now we can set the powers
//            robot.FLMotor.setPower(wheelPwr[0]);
//            robot.BLMotor.setPower(wheelPwr[1]);
//            robot.FRMotor.setPower(wheelPwr[2]);
//            robot.BRMotor.setPower(wheelPwr[3]);
//
//        }

    }

    /**
     * Calculate the power in the x direction
     * @param desiredAngle angle on the x axis
     * @param speed robot's speed
     * @return the x vector
     */
    private double calculateX(double desiredAngle, double speed) {
        return Math.sin(Math.toRadians(desiredAngle)) * speed;
    }

    /**
     * Calculate the power in the y direction
     * @param desiredAngle angle on the y axis
     * @param speed robot's speed
     * @return the y vector
     */
    private double calculateY(double desiredAngle, double speed) {
        return Math.cos(Math.toRadians(desiredAngle)) * speed;
    }


    /**
     * Takes the horizontal power, vertical power, and pivoting power and determine how much power to apply to each wheel, and normalizes the max poer
     * @param horizontal the horizontal (x vector) power
     * @param vertical the vertical (y vector) power
     * @param pivot the pivoting power
     * @param maxPower the max power the wheels can move
     */
    public double[] rawSlide(double horizontal, double vertical, double pivot, double maxPower){
        //create an array with all the speeds
        double powers[] = {vertical+horizontal+pivot, vertical-horizontal+pivot, vertical-horizontal-pivot, vertical+horizontal-pivot};

        //Only adjust speeds if the robot is moving
        if(horizontal!=0 || vertical!=0){
            int max = 0;
            int counter = 0;

            //determine the maximum speed out of the four motors
            for(double element:powers){
                if(Math.abs(element)>Math.abs(powers[max])){
                    max = counter;
                }
                counter++;
            }

            //set the maximum as a variable
            double maxCalculatedPower = Math.abs(powers[max]);

            //divide all of the speeds by the max speed to make sure that
            if(maxCalculatedPower!=0){
                powers[0]=powers[0]/maxCalculatedPower*maxPower;
                powers[1]=powers[1]/maxCalculatedPower*maxPower;
                powers[2]=powers[2]/maxCalculatedPower*maxPower;
                powers[3]=powers[3]/maxCalculatedPower*maxPower;
            }
        }
        return powers;
    }

}