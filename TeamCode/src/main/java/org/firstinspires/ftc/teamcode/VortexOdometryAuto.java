package org.firstinspires.ftc.teamcode;

import android.os.SystemClock;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;


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

import HelperClass.Constants;
import HelperClass.OdometryRobot;
//import HelperClass.PID;
import HelperClass.Robot;
import HelperClass.RobotVision;


@Autonomous(name="OdometryAuto", group="Linear Opmode")
public class VortexOdometryAuto extends LinearOpMode {

    static final double threshold = 0.3;
    private ElapsedTime runtime = new ElapsedTime();
    private double currentTime = runtime.seconds();
    private double rightx = 0;
    private double leftx = 0;
    private double lefty = 0;

    double distanceToTargetX = 0;
    double distanceToTargetY = 0;
    double distance = 0;
    double targetX = 0;
    double targetY = 0;
    double sideTargetX = 0;
    double sideTargetY = 0;
    double yAxisDistInch = 0;
    double xAxisDistInch = 0;
    int gotoTarget = 1;
    boolean move_fwd = false;
    boolean move_side = false;
    boolean move_side_right = false;
    boolean move_back = false;
    public double fwdpwr = 0.7;
    double LeftDist = 0;
    double fwdDist = 0;
    double backDist = 0;
    double rightDist = 0;
    double fwdDist1 = 0;
    double startXPos, startYPos;


    //enum vars defined
    private enum Step {
        confirmation, startMv, mvForward, robotTurn, checkRings, mvBack, mvLeft, mvRight, mvStrafe, dropWobble, shootRing,
        launchLine, quad, single, none, angleAdj, park, stop, turnL, deliverWobble, testMv
    }

    private Step step = Step.confirmation;

    public RobotVision rVision;
    public String ringCountOnField = null;
    public String ringCountOnFieldLast = null;
    private boolean ringFoundConfirmed = false;

    OdometryRobot odoRobot = new OdometryRobot();

    OdometryCoordinatePosition globalPositionUpdate;

    @Override
    public void runOpMode() throws InterruptedException {

        odoRobot.init(hardwareMap);
        rVision = new RobotVision();
        rVision.initVuforia(hardwareMap);
        rVision.initTfod(hardwareMap);
        rVision.activateTFOD();
        runtime.reset();

        //Create and start GlobalCoordinatePosition thread to constantly update the global coordinate positions

        OdometryCoordinatePosition globalPositionUpdate = new OdometryCoordinatePosition(odoRobot.leftEncoderMotor, odoRobot.rightEncoderMotor, odoRobot.centerEncoderMotor);
        Thread positionThread = new Thread(globalPositionUpdate);
        positionThread.start();

        // ensure correct ring detection
        while (!opModeIsActive() && !isStopRequested()) {
            rVision.updateRingCount();
            ringCountOnField = rVision.getRingCount();
            ringCountOnFieldLast = ringCountOnField;
            odoRobot.claw.setPosition(1);
            telemetry.addData("RingCount", ringCountOnField);
            telemetry.addData("Last Ring Count", ringCountOnFieldLast);
            telemetry.addData("Ring Count Confirmed", ringFoundConfirmed);
            telemetry.addData("distanceToTargetX ", distanceToTargetX / odoRobot.countsPerInch);
            telemetry.addData("distanceToTargetY ", distanceToTargetY / odoRobot.countsPerInch);
            telemetry.addData("X Position", globalPositionUpdate.returnXCoordinate() / odoRobot.countsPerInch);
            telemetry.addData("Y Position", globalPositionUpdate.returnYCoordinate() / odoRobot.countsPerInch);
            telemetry.update();
        }


        waitForStart();

        // start is pressed, robot is active


        while (opModeIsActive()) {
            // start the time
            currentTime = runtime.seconds();

            switch (step) {
                case confirmation:
                    //if (ringCountOnField == ringCountOnFieldLast && ringFoundConfirmed) {

                    ringCountOnField = "Quad";

                    step = Step.mvForward;
                    //}
                    break;


                // 1. Go forward by X distance
                // 2. Strafe for Y distance - needed only for B
                // 3. Drop wobble goal
                // 4. Back to shooting line and
                // 5. Start shooter and push rings
                // 6. turn around pick up ring
                // 7. Park the robot

                case mvForward:

                    move_fwd = true;

                    startXPos = globalPositionUpdate.returnXCoordinate() / odoRobot.countsPerInch;
                    startYPos = globalPositionUpdate.returnYCoordinate() / odoRobot.countsPerInch;

                    if (ringCountOnField == "Quad") {
                        LeftDist = 15;
                        fwdDist = 95;
                        backDist = 65;
                        rightDist = 0;

                    } else if (ringCountOnField == "Single") {
                        LeftDist = 15;
                        fwdDist = 75;
                        backDist = 70;
                        rightDist = 0;

                    } else if (ringCountOnField == "Zero") {
                        LeftDist = 18;
                        fwdDist = 40;
                        backDist = 00;
                        rightDist = 0;
                        fwdDist1 = 5; // go forward to shoot
                    }

                    globalPositionUpdate.reverseRightEncoder();
                    globalPositionUpdate.reverseLeftEncoder();
                    SystemClock.sleep(100);

                    xAxisDistInch = globalPositionUpdate.returnXCoordinate() / odoRobot.countsPerInch;
                    yAxisDistInch = globalPositionUpdate.returnYCoordinate() / odoRobot.countsPerInch;

                    targetX = 0;
                    targetY = fwdDist + yAxisDistInch;

                    moveToPoint(globalPositionUpdate, targetX, targetY, 0.8, 0.0, 00.0);

                    // 2. Strafe for X distance for B or else move to drop wobble
                    if (yAxisDistInch >= targetY && ringCountOnField == "Single") {
                        step = Step.mvLeft;
                        move_fwd = false;
                        move_side = true;
                        odoRobot.robotStop();
                        SystemClock.sleep(200);

                    } else if (yAxisDistInch >= targetY) {
                        step = Step.dropWobble;
                        move_fwd = false;
                        odoRobot.robotStop();
                        SystemClock.sleep(200);

                    }
                    break;


                case dropWobble:
                    targetY = 0;
                    targetX = 0;
                    odoRobot.arm.setTargetPosition(200);
                    odoRobot.arm.setPower(-1);
                    SystemClock.sleep(200);
                    odoRobot.arm.setPower(-0.2);
                    SystemClock.sleep(250);
                    odoRobot.arm.setPower(0);
                    SystemClock.sleep(1250);
                    odoRobot.claw.setPosition(-1);
                    SystemClock.sleep(1000);
                    //upwards
                    odoRobot.arm.setTargetPosition(0);
                    odoRobot.arm.setPower(0.5);
                    SystemClock.sleep(300);
                    odoRobot.arm.setPower(0.3);
                    SystemClock.sleep(200);
                    odoRobot.arm.setPower(0);
                    SystemClock.sleep(100);
                    odoRobot.claw.setPosition(1);
                    SystemClock.sleep(100);
                    odoRobot.robotStop();

                    // 4. Now back to shooting line , move backwards
                    // code to drop wobble goal in the next block
                    if (ringCountOnField == "Quad" ||  ringCountOnField == "Single") {
                        step = Step.mvBack;

                    } else if (ringCountOnField == "Zero") {
                        step = Step.stop;
                    }
                    break;

                case mvBack:
                    // 4. Now back to shooting line , move backwards
                    // code to drop wobble goal in the next block


                    globalPositionUpdate.reverseRightEncoder();
                    globalPositionUpdate.reverseLeftEncoder();
                    SystemClock.sleep(100);
                    xAxisDistInch = globalPositionUpdate.returnXCoordinate() / odoRobot.countsPerInch;
                    yAxisDistInch = globalPositionUpdate.returnYCoordinate() / odoRobot.countsPerInch;

                    targetX = 00;
                    targetY = -backDist;
                    move_back = true;



                    moveToPoint(globalPositionUpdate, targetX, targetY, -0.7, 0.0, 00.0);

                    if (yAxisDistInch >= targetY)  {
                        move_side = true;
                        move_back = false;
                        odoRobot.robotStop();
                        SystemClock.sleep(500);
                    }

                    if( ringCountOnField == "Single") {
                        step = Step.shootRing;
                    } else {
                        step = Step.mvLeft;
                    }

//                    telemetry.addData("1- Y Position", yAxisDistInch);
//                    telemetry.update();
//                    SystemClock.sleep(2000);



                    break;

                case mvLeft:
                    // 4. Now move left to align with high goal shooting line

                    xAxisDistInch = globalPositionUpdate.returnXCoordinate() / odoRobot.countsPerInch;
                    yAxisDistInch = globalPositionUpdate.returnYCoordinate() / odoRobot.countsPerInch;

                    sideTargetX = xAxisDistInch - LeftDist;
                    sideTargetY = 0;
                    moveLeft(globalPositionUpdate, sideTargetX, sideTargetY, -0.5, 0.0, 00.0);



                    if (xAxisDistInch <= sideTargetX)  {
                        odoRobot.robotStop();
                    }

                    if( ringCountOnField == "Single") {
                        step = Step.dropWobble;
                    } else {
                        step = Step.shootRing;
                    }

                    break;

                case shootRing:


                    odoRobot.shooting1.setPower(-0.5);
                    odoRobot.shooting2.setPower(-0.5);
                    SystemClock.sleep(2500);

                    for (int i =1; i<=3; i++){
                        odoRobot.ringPush.setPosition(-1);
                        SystemClock.sleep(1000);
                        odoRobot.ringPush.setPosition(1);
                        SystemClock.sleep(1000);
                    }
                    odoRobot.shooting1.setPower(0);
                    odoRobot.shooting2.setPower(0);

                    step = Step.park;

                    break;


                case park:

                    globalPositionUpdate.reverseRightEncoder();
                    globalPositionUpdate.reverseLeftEncoder();
                    SystemClock.sleep(100);

                    xAxisDistInch = globalPositionUpdate.returnXCoordinate() / odoRobot.countsPerInch;
                    yAxisDistInch = globalPositionUpdate.returnYCoordinate() / odoRobot.countsPerInch;

                    targetX = 0;
                    targetY = 10 + yAxisDistInch;

                    moveToPoint(globalPositionUpdate, targetX, targetY, 0.8, 0.0, 00.0);
                    if (yAxisDistInch >= targetY) {
                        step = Step.stop;
                        odoRobot.robotStop();
                        SystemClock.sleep(200);
                    }

                    break;
                //robot.moveUpdate();

                case stop:
                    odoRobot.FLMotor.setPower(0);
                    odoRobot.BLMotor.setPower(0);
                    odoRobot.FRMotor.setPower(0);
                    odoRobot.BRMotor.setPower(0);
                    break;
                //robot.moveUpdate();
            }

//            if (ringCountOnField == null) {
//                rVision.updateRingCount();
//                ringCountOnField = rVision.getRingCount();
//                SystemClock.sleep(2000);
//                if (currentTime > 3.0) {
//                    ringCountOnField = "Zero";
//                    ringCountOnFieldLast = ringCountOnField;
//                    ringFoundConfirmed = true;
//
//                } else {
//                    rVision.updateRingCount();
//                    ringCountOnField = rVision.getRingCount();
//                }
//            } else {
//                ringFoundConfirmed = true;
//            }



            telemetry.addData("Current Time", "%s", currentTime);
            telemetry.addData("VortexStep", "%s", step);
            telemetry.addData("Distance ", distance / odoRobot.countsPerInch);

            telemetry.addData("distanceToTargetX ", distanceToTargetX / odoRobot.countsPerInch);
            telemetry.addData("distanceToTargetY ", distanceToTargetY / odoRobot.countsPerInch);

            telemetry.addData("RingCount", "%s", ringCountOnField);
            telemetry.addData("Last Ring Count", "%s", ringCountOnFieldLast);
            telemetry.addData("Ring Count Confirmed", "%s", ringFoundConfirmed);

            telemetry.addData("X Position", globalPositionUpdate.returnXCoordinate() / odoRobot.countsPerInch);
            telemetry.addData("Y Position", globalPositionUpdate.returnYCoordinate() / odoRobot.countsPerInch);
            telemetry.addData("Odometry Angle (Degree)", globalPositionUpdate.returnAngleDegree());
            telemetry.addData("IMU Robot Angle: ", "%f", odoRobot.imuRobotAngle());
            telemetry.addData("Vertical left encoder position", odoRobot.leftEncoderMotor.getCurrentPosition());
            telemetry.addData("Vertical right encoder position", odoRobot.rightEncoderMotor.getCurrentPosition());
            telemetry.addData("horizontal encoder position", odoRobot.centerEncoderMotor.getCurrentPosition());
            telemetry.update();
        }

        globalPositionUpdate.stop();
    }

    // Moving with adjusting the speed based on turns & angle
    public void moveToPoint (OdometryCoordinatePosition ngpu,double tX, double tY,
                             double robotPwr, double turnPwr, double desiredOrientation){

        double fl, fr, bl, br;

        distanceToTargetX = tX * odoRobot.countsPerInch - ngpu.returnXCoordinate();
        distanceToTargetY = tY * odoRobot.countsPerInch - ngpu.returnYCoordinate();
        xAxisDistInch = ngpu.returnXCoordinate() / odoRobot.countsPerInch;
        yAxisDistInch = ngpu.returnYCoordinate() / odoRobot.countsPerInch;


//        telemetry.addData("2- Y Position", yAxisDistInch);
//        telemetry.update();
//        SystemClock.sleep(2000);
        //while ((yAxisDistInch < tY && move_fwd == true) || (yAxisDistInch < tY && move_back == true)) {
        while (yAxisDistInch < tY ) {


            distanceToTargetX = tX * odoRobot.countsPerInch - ngpu.returnXCoordinate();
            distanceToTargetY = tY * odoRobot.countsPerInch - ngpu.returnYCoordinate();
            double absoluteAngleToPoint = Math.atan2(distanceToTargetY, distanceToTargetX);
            xAxisDistInch = ngpu.returnXCoordinate() / odoRobot.countsPerInch;
            yAxisDistInch = ngpu.returnYCoordinate() / odoRobot.countsPerInch;

            double robotAngleToPoint = ngpu.returnAngleRadian();

            //double robotMovementAngle = Math.toDegrees(Math.atan2(distanceToTargetX, distanceToTargetY));

            //turnPID.run(0, robotAngleToPoint);
            //double anglePwr = turnPID.getOutput() * turnPwr;

            double x = Math.cos(absoluteAngleToPoint) * robotPwr;
            double y = Math.sin(absoluteAngleToPoint) * robotPwr;

            double pivotCorrection = desiredOrientation - ngpu.returnAngleDegree();

            x *= -1.0;

            double power = Math.hypot(x, y);
            double theta = Math.atan2(y, x) - ngpu.returnAngleRadian();

            fl = (Math.cos(theta - (Math.PI / 4))) * power + turnPwr;
            fr = (Math.sin(theta + (Math.PI / 4))) * power - turnPwr;
            bl = (Math.sin(theta + (Math.PI / 4))) * power + turnPwr;
            br = (Math.cos(theta - (Math.PI / 4))) * power - turnPwr;

            odoRobot.FLMotor.setPower(fl);
            odoRobot.BLMotor.setPower(bl);
            odoRobot.FRMotor.setPower(fr);
            odoRobot.BRMotor.setPower(br);


        }
    }


    // Moving with adjusting the speed based on turns & angle
    public void moveRight (OdometryCoordinatePosition ngpu,double tX, double tY,
                           double robotPwr, double turnPwr, double desiredOrientation){
        double fl, fr, bl, br;


        distanceToTargetX = tX * odoRobot.countsPerInch - ngpu.returnXCoordinate();
        distanceToTargetY = tY * odoRobot.countsPerInch - ngpu.returnYCoordinate();
        xAxisDistInch = ngpu.returnXCoordinate() / odoRobot.countsPerInch;
        yAxisDistInch = ngpu.returnYCoordinate() / odoRobot.countsPerInch;

        //telemetry.addData("2- X Position", xAxisDistInch);
        //telemetry.addData("2- TargetX ", tX);
        //telemetry.update();
        //SystemClock.sleep(2000);
        //while (xAxisDistInch < targetX) { // This line woks for right turn

        while (xAxisDistInch < tX) {

            distanceToTargetX = tX * odoRobot.countsPerInch - ngpu.returnXCoordinate();
            distanceToTargetY = tY * odoRobot.countsPerInch - ngpu.returnYCoordinate();
            double absoluteAngleToPoint = Math.atan2(distanceToTargetY, distanceToTargetX);
            xAxisDistInch = ngpu.returnXCoordinate() / odoRobot.countsPerInch;
            yAxisDistInch = ngpu.returnYCoordinate() / odoRobot.countsPerInch;

            double robotAngleToPoint = ngpu.returnAngleRadian();

            double x = Math.cos(absoluteAngleToPoint) * robotPwr;
            double y = Math.sin(absoluteAngleToPoint) * robotPwr;

            double pivotCorrection = desiredOrientation - ngpu.returnAngleDegree();

            x *= -1.0;

            double power = Math.hypot(x, y);
            double theta = Math.atan2(y, x) - ngpu.returnAngleRadian();

            fl = (Math.cos(theta - (Math.PI / 4))) * power + turnPwr;
            fr = (Math.sin(theta + (Math.PI / 4))) * power - turnPwr;
            bl = (Math.sin(theta + (Math.PI / 4))) * power + turnPwr;
            br = (Math.cos(theta - (Math.PI / 4))) * power - turnPwr;

//            odoRobot.FLMotor.setPower(fl);
//            odoRobot.BLMotor.setPower(-bl);
//            odoRobot.FRMotor.setPower(-fr);
//            odoRobot.BRMotor.setPower(br);

            odoRobot.FLMotor.setPower(robotPwr);
            odoRobot.BLMotor.setPower(-robotPwr);
            odoRobot.FRMotor.setPower(-robotPwr);
            odoRobot.BRMotor.setPower(robotPwr);
        }
    }


    // Moving with adjusting the speed based on turns & angle
    public void moveLeft (OdometryCoordinatePosition ngpu,double tX, double tY,
                          double robotPwr, double turnPwr, double desiredOrientation){
        double fl, fr, bl, br;
        double x, y , robotAngleToPoint, pivotCorrection;

        distanceToTargetX = tX * odoRobot.countsPerInch - ngpu.returnXCoordinate();
        distanceToTargetY = tY * odoRobot.countsPerInch - ngpu.returnYCoordinate();
        xAxisDistInch = ngpu.returnXCoordinate() / odoRobot.countsPerInch;
        yAxisDistInch = ngpu.returnYCoordinate() / odoRobot.countsPerInch;

//        telemetry.addData("2- X Position", xAxisDistInch);
//        telemetry.addData("2- TargetX ", tX);
//        telemetry.update();
//        SystemClock.sleep(2000);
        //while (xAxisDistInch < targetX) { // This line woks for right turn

        while (xAxisDistInch > tX) {

            distanceToTargetX = tX * odoRobot.countsPerInch - ngpu.returnXCoordinate();
            distanceToTargetY = tY * odoRobot.countsPerInch - ngpu.returnYCoordinate();
            double absoluteAngleToPoint = Math.atan2(distanceToTargetY, distanceToTargetX);
            xAxisDistInch = ngpu.returnXCoordinate() / odoRobot.countsPerInch;
            yAxisDistInch = ngpu.returnYCoordinate() / odoRobot.countsPerInch;

            robotAngleToPoint = ngpu.returnAngleRadian();

//            x = Math.cos(absoluteAngleToPoint) * robotPwr;
//            y = Math.sin(absoluteAngleToPoint) * robotPwr;
//
//            pivotCorrection = desiredOrientation - ngpu.returnAngleDegree();
//
//            x *= -1.0;
//
//            double power = Math.hypot(x, y);
//            double theta = Math.atan2(y, x) - ngpu.returnAngleRadian();
//
//            fl = (Math.cos(theta - (Math.PI / 4))) * power + turnPwr;
//            fr = (Math.sin(theta + (Math.PI / 4))) * power - turnPwr;
//            bl = (Math.sin(theta + (Math.PI / 4))) * power + turnPwr;
//            br = (Math.cos(theta - (Math.PI / 4))) * power - turnPwr;

//            odoRobot.FLMotor.setPower(fl);
//            odoRobot.BLMotor.setPower(-bl);
//            odoRobot.FRMotor.setPower(-fr);
//            odoRobot.BRMotor.setPower(br);

            odoRobot.FLMotor.setPower(robotPwr);
            odoRobot.BLMotor.setPower(-robotPwr);
            odoRobot.FRMotor.setPower(-robotPwr);
            odoRobot.BRMotor.setPower(robotPwr);
        }
    }


    /**
     * Calculate the power in the x direction
     *
     * @param desiredAngle angle on the x axis
     * @param speed        robot's speed
     * @return the x vector
     */
    private double calculateX ( double desiredAngle, double speed){
        return Math.cos(Math.toRadians(desiredAngle)) * speed;
    }

    /**
     * Calculate the power in the y direction
     *
     * @param desiredAngle angle on the y axis
     * @param speed        robot's speed
     * @return the y vector
     */
    private double calculateY ( double desiredAngle, double speed){
        return Math.sin(Math.toRadians(desiredAngle)) * speed;
    }





}














