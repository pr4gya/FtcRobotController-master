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

//    private PID turnPID = new PID(1.8, 0.1, 1, 2.0, 0.05, -0.3, 0.3);

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
        runtime.reset();
        telemetry.addData("status", "initialized");
        telemetry.update();

        //Create and start GlobalCoordinatePosition thread to constantly update the global coordinate positions
        OdometryCoordinatePosition globalPositionUpdate = new OdometryCoordinatePosition(odoRobot.leftEncoderMotor, odoRobot.rightEncoderMotor, odoRobot.centerEncoderMotor);
        Thread positionThread = new Thread(globalPositionUpdate);
        positionThread.start();

        // ensure correct ring detection
        while (!opModeIsActive() && !isStopRequested()) {
            // Now get the number of rings on the field
//            rVision.updateRingCount();
//            ringCountOnField = rVision.getRingCount();
//            ringCountOnFieldLast = ringCountOnField;
//
//            int correct = robot.arm.getCurrentPosition();
//            robot.arm.setPower(0);
//            robot.arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//            robot.claw.setPosition(-1);
//            robot.ringPush.setPosition(0);
//
//            telemetry.addData("RingCount", "%s", ringCountOnField);
//            telemetry.addData("Robot Angle-Deg", "%.1f", robot.getRobotAngle());
//            telemetry.addData("Robot Angle-Rad", "%.1f", robot.getRobotAngleRad());
//            telemetry.addData("FLMotor-1", "%d ", robot.FLMotor.getCurrentPosition());
//            telemetry.addData("arm position", "%s", correct);
//            telemetry.update();
        }


        waitForStart();

        // start is pressed, robot is active


        while (opModeIsActive()) {
            // start the time
            currentTime = runtime.seconds();

            switch (step) {
                case confirmation:
                    //if (ringCountOnField == Constants.RING_COUNT_FOUR || ringCountOnField == Constants.RING_COUNT_ONE
                    //|| ringCountOnField == Constants.RING_COUNT_ZERO ) {
                    //}
                    ringCountOnField = "Quad";
                    step = Step.mvForward;
                    break;

//                case testMv:
//                    // Goto Position A on the filed to deliver wobble goal
//                    move_fwd = true;
//                    targetX = 06;
//                    targetY = 36;
//                    distanceToTargetX = targetX * odoRobot.countsPerInch - globalPositionUpdate.returnXCoordinate();
//                    distanceToTargetY = targetY * odoRobot.countsPerInch - globalPositionUpdate.returnYCoordinate();
//                    distance = Math.hypot(distanceToTargetX, distanceToTargetY);
//                    //gotoPosition(globalPositionUpdate, targetX, targetY, 0.3, 0.0, 1.0 );
//
//                    moveToPoint(globalPositionUpdate, targetX, targetY, 0.8, 0.12, 00.0);
//                    if (distanceToTargetY < 0) {
//                        step = Step.robotTurn;
//                    }
//                    break;

                // 1. Go forward by X distance
                // 2. Strafe for Y distance - needed only for B
                // 3. Drop wobble goal
                // 4. Back to shooting line and
                // 5. Start shooter and push rings
                // 6. turn around pick up ring
                // 7. Park the robot

                case mvForward:
                    targetX = 00;
                    targetY = 87;
                    move_fwd = true;
                    //moveToPoint(globalPositionUpdate, targetX, targetY, 1.0, 0.0, 00.0);
                    globalPositionUpdate.reverseRightEncoder();
                    globalPositionUpdate.reverseLeftEncoder();
                    SystemClock.sleep(100);
                    moveToPoint(globalPositionUpdate, targetX, targetY, 0.6, 0.0, 00.0);

                    // 2. Strafe for X distance for B or else move to drop wobble
                    if (yAxisDistInch >= targetY && ringCountOnField == "Single") {
                        step = Step.mvStrafe;
                        move_fwd = false;
                    } else if (yAxisDistInch >= targetY ) {
                        step = Step.dropWobble;
//                        step = Step.mvBack;
                        move_fwd = false;
                        odoRobot.robotStop();

                    }
                    break;

                case mvStrafe:
                    targetX = -12;
                    targetY = 0;
                    move_fwd = false;
                    move_side = true;
                    //moveToPoint(globalPositionUpdate, targetX, targetY, 0.8, 0.0, 00.0);
                    if (distanceToTargetX < 0 ) {
                        step = Step.dropWobble;
                    }
                    break;

                case dropWobble :
                    targetY = 00;
                    targetX = 00;

                    // 3. Drop wobble goal
                    // code to drop wobble goal in the next block

                    // 4. Now back to shooting line , move backwards
                    // code to drop wobble goal in the next block
                    step = Step.mvBack;
                    break;

                case mvBack:
                    // 4. Now back to shooting line , move backwards
                    // code to drop wobble goal in the next block
                    if (ringCountOnField == "Quad") {
                        targetX = 00;
                        targetY = -60;
                        move_back = true;
                        globalPositionUpdate.reverseRightEncoder();
                        globalPositionUpdate.reverseLeftEncoder();
                        SystemClock.sleep(100);

                        //move_fwd = false;
                        //move_side = false;
                        //move_back = true;
                        //SystemClock.sleep(2000);
                        //distanceToTargetX = targetX * odoRobot.countsPerInch - globalPositionUpdate.returnXCoordinate();
                        //distanceToTargetY = targetY * odoRobot.countsPerInch - globalPositionUpdate.returnYCoordinate();
                        xAxisDistInch = globalPositionUpdate.returnXCoordinate() / odoRobot.countsPerInch;
                        yAxisDistInch = globalPositionUpdate.returnYCoordinate() / odoRobot.countsPerInch;

                        moveToPoint(globalPositionUpdate, targetX, targetY, -0.6, 0.0, 00.0);
                        //telemetry.addData("1- Y Position", yAxisDistInch);
                        //telemetry.update();
                        //SystemClock.sleep(2000);
                        //yAxisDistInch < targetY
                        if (yAxisDistInch >= targetY)  {
                            step = Step.mvLeft;
                            move_back = false;
                            odoRobot.robotStop();
                        }
                        break;
                    }
                    break;

                case mvLeft:
                    // 4. Now move left to align with high goal shooting line
                    sideTargetX = 15;
                    sideTargetY = 0;
                    //move_back = true;
//                    telemetry.addData("10- X Position", xAxisDistInch);
//                    telemetry.update();
//                    SystemClock.sleep(2000);
                    //globalPositionUpdate.reverseCenterEncoder();
                    //SystemClock.sleep(50);
//                    telemetry.addData("11- X Position", xAxisDistInch);
//                    telemetry.update();
//                    SystemClock.sleep(2000);

                    xAxisDistInch = globalPositionUpdate.returnXCoordinate() / odoRobot.countsPerInch;
                    yAxisDistInch = globalPositionUpdate.returnYCoordinate() / odoRobot.countsPerInch;
                    moveLeft(globalPositionUpdate, sideTargetX, sideTargetY, -0.4, 0.0, 00.0);

//                    telemetry.addData("1- X Position", xAxisDistInch);
//                    telemetry.update();
//                    SystemClock.sleep(2000);

                    if (xAxisDistInch <= -sideTargetX)  {
                        step = Step.stop;
                        odoRobot.robotStop();
                    }
                    break;

                case mvRight:
                    // 4. Now move left to align with high goal shooting line

                    sideTargetX = 15;
                    sideTargetY = 0;
                    //move_back = true;

                    globalPositionUpdate.reverseRightEncoder();
                    globalPositionUpdate.reverseLeftEncoder();
                    SystemClock.sleep(100);

                    xAxisDistInch = globalPositionUpdate.returnXCoordinate() / odoRobot.countsPerInch;
                    yAxisDistInch = globalPositionUpdate.returnYCoordinate() / odoRobot.countsPerInch;

                    moveRight(globalPositionUpdate, sideTargetX, sideTargetY, 0.5, 0.0, 00.0);
                    telemetry.addData("1- X Position", xAxisDistInch);
                    telemetry.update();
                    SystemClock.sleep(2000);

                    if (xAxisDistInch >= sideTargetX)  {
                        step = Step.stop;
                        odoRobot.robotStop();
                    }
                    break;

                case shootRing:
                    // 4. Now back to shooting line , move backwards
                    // code to drop wobble goal in the next block


                    break;


                case robotTurn:
                    if (step == Step.robotTurn) {
                        double dAngle = 200;
                        //odoRobot.robotTurn(tRight, dAngle, tAngle, fwdPwr);
                        break;
                    }


                    if (globalPositionUpdate.returnAngleDegree() < 0) {
                        step = Step.robotTurn;
                    }
                    break;
                case stop:
                    odoRobot.FLMotor.setPower(0);
                    odoRobot.BLMotor.setPower(0);
                    odoRobot.FRMotor.setPower(0);
                    odoRobot.BRMotor.setPower(0);
                    break;
                //robot.moveUpdate();
            }

            //robot.moveUpdate();

            telemetry.addData("Current Time", "%s", currentTime);
            telemetry.addData("VortexStep", "%s", step);
            telemetry.addData("Distance ", distance / odoRobot.countsPerInch);

            telemetry.addData("distanceToTargetX ", distanceToTargetX / odoRobot.countsPerInch);
            telemetry.addData("distanceToTargetY ", distanceToTargetY / odoRobot.countsPerInch);

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

        // This only supports moving to Y direction

        // 0 ... 80 and target is 80
        // 85 ....50 and target is 45

//        telemetry.addData("2- Y Position", yAxisDistInch);
//        telemetry.update();
//        SystemClock.sleep(2000);
        while ((yAxisDistInch < tY && move_fwd == true) || (yAxisDistInch < tY && move_back == true)) {

            //while ((y_axisDistanceInInch < targetY && move_fwd == true) || (y_axisDistanceInInch > targetY && move_back == true)) {

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

        while (xAxisDistInch > -tX) {

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


    /**
     * Takes the horizontal power, vertical power, and pivoting power and determine how much power to apply to each wheel, and normalizes the max poer
     *
     * @param horizontal the horizontal (x vector) power
     * @param vertical   the vertical (y vector) power
     * @param pivot      the pivoting power
     * @param maxPower   the max power the wheels can move
     */
    public double[] rawSlide ( double horizontal, double vertical, double pivot, double maxPower)
    {
        //create an array with all the speeds
        double powers[] = {vertical + horizontal + pivot, vertical - horizontal + pivot, vertical - horizontal - pivot, vertical + horizontal - pivot};

        //Only adjust speeds if the robot is moving
        if (horizontal != 0 || vertical != 0) {
            int max = 0;
            int counter = 0;

            //determine the maximum speed out of the four motors
            for (double element : powers) {
                if (Math.abs(element) > Math.abs(powers[max])) {
                    max = counter;
                }
                counter++;
            }

            //set the maximum as a variable
            double maxCalculatedPower = Math.abs(powers[max]);

            //divide all of the speeds by the max speed to make sure that
            if (maxCalculatedPower != 0) {
                powers[0] = powers[0] / maxCalculatedPower * maxPower;
                powers[1] = powers[1] / maxCalculatedPower * maxPower;
                powers[2] = powers[2] / maxCalculatedPower * maxPower;
                powers[3] = powers[3] / maxCalculatedPower * maxPower;
            }
        }
        return powers;
    }


}














