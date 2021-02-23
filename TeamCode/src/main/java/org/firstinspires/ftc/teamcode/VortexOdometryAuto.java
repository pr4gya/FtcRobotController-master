package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import android.os.SystemClock;
import HelperClass.OdometryRobot;
import HelperClass.RobotVision;
import com.qualcomm.robotcore.hardware.DcMotor;
@Autonomous(name="odometry autonomous", group="Linear Opmode")
public class VortexOdometryAuto extends LinearOpMode {
    //calls helper classes
    public RobotVision rVision;
    // public Robot robot;
    OdometryRobot robot = new OdometryRobot();
    OdometryRobot odoRobot = new OdometryRobot();
    OdometryCoordinatePosition globalPositionUpdate;
    //ring count vars
    public String ringCountOnField = null;
    public String ringCountOnFieldLast = null;
    private boolean ringFoundConfirmed = false;
    //odo vars
    private double rightx = 0;
    private double leftx = 0;
    private double lefty = 0;
    private double distanceToTargetX = 0;
    private double distanceToTargetY = 0;
    double distance = 0;
    double targetX = 0;
    double targetY = 1;
    int gotoTarget = 0;
    boolean move_fwd = false;
    boolean move_side = false;

    //turn vars
    boolean tRight;
    double tAngle;
    double dAngle;
    double fwdPwr;

    //time
    private ElapsedTime runtime = new ElapsedTime();
    private double currentTime = runtime.seconds();
    private double startTime;
    //vars
    public double fwdpwr = 0.7;
    //enum vars defined
    private enum Step {confirmation, startMv, turnL, turnR, checkRings, mvBack, mvForward, mvStrafe, dropWobble, shootRing,
        launchLine, quad, single, none, angleAdj, park, stop, robotTurn
    }
    private Step step = Step.mvForward;
    @Override
    public void runOpMode() throws InterruptedException {
        runtime.reset();
        startTime = runtime.seconds();
        odoRobot.init(hardwareMap);
        /* Enable for vision
        rVision = new RobotVision();
        rVision.initVuforia(hardwareMap);
        rVision.initTfod(hardwareMap);
        rVision.activateTFOD();
        */
        while (!opModeIsActive() && !isStopRequested()) {
            telemetry.addData("Robot Angle", "%.1f", odoRobot.imuRobotAngle());
            telemetry.addData("Ring Count", "%s", ringCountOnField);
            telemetry.addData("Ring Count", "%s", ringCountOnFieldLast);
            telemetry.update();
            /*
            rVision.updateRingCount();
            ringCountOnFieldLast = rVision.getRingCount();
            rVision.updateRingCount();
            ringCountOnField = rVision.getRingCount();
            */
            int armpos = odoRobot.arm.getCurrentPosition();
            odoRobot.arm.setTargetPosition(armpos);
            odoRobot.arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            odoRobot.claw.setPosition(0);
        }
        //Create and start GlobalCoordinatePosition thread to constantly update the global coordinate positions
        OdometryCoordinatePosition globalPositionUpdate = new OdometryCoordinatePosition(odoRobot.leftEncoderMotor, odoRobot.rightEncoderMotor, odoRobot.centerEncoderMotor);
        Thread positionThread = new Thread(globalPositionUpdate);
        positionThread.start();
        //gotoPosition(0 * robot.countsPerInch, 24 * robot.countsPerInch, 0.5, 0 , 1.0 * robot.countsPerInch);
        globalPositionUpdate.reverseRightEncoder();
        globalPositionUpdate.reverseLeftEncoder();
        //globalPositionUpdate.reverseCenterEncoder();
        move_fwd = true;
        targetY = 60;
        distanceToTargetX = targetX * odoRobot.countsPerInch - globalPositionUpdate.returnXCoordinate();
        distanceToTargetY = targetY * odoRobot.countsPerInch - globalPositionUpdate.returnYCoordinate();
        distance = Math.hypot(distanceToTargetX, distanceToTargetY);


        waitForStart();


        while(opModeIsActive()) {
            runtime.reset();
            currentTime = runtime.seconds();
            switch (step) {
                case confirmation:
                case none:
//                    if (ringCountOnField == ringCountOnFieldLast && ringFoundConfirmed) {
//                        step = Step.startMv;
//                    }
                    break;
                case startMv:
                    while (step == Step.startMv && distanceToTargetY > 0 && move_fwd) {
                        //double[] wheelPwr = rawSlide(0.0, 0.2, 0.0, 1.0);
                        odoRobot.FLMotor.setPower(fwdpwr);
                        odoRobot.BLMotor.setPower(fwdpwr);
                        odoRobot.FRMotor.setPower(fwdpwr);
                        odoRobot.BRMotor.setPower(fwdpwr);
                        distanceToTargetX = targetX * odoRobot.countsPerInch - globalPositionUpdate.returnXCoordinate();
                        distanceToTargetY = targetY * odoRobot.countsPerInch - globalPositionUpdate.returnYCoordinate();
                        distance = Math.hypot(distanceToTargetX, distanceToTargetY);
                    }
                    step = Step.turnL;
                    break;
                case turnL:
                    while (step == Step.turnL && currentTime<3) {
                        odoRobot.FLMotor.setPower(-fwdpwr);
                        odoRobot.BLMotor.setPower(fwdpwr);
                        odoRobot.FRMotor.setPower(-fwdpwr);
                        odoRobot.BRMotor.setPower(fwdpwr);
                    }
                    step = Step.stop;
                    break;

                case mvForward:
                // Goto Position A on the filed to deliver wobble goal
                    move_fwd = true;
                    targetX = 06;
                    targetY = 36;
                    distanceToTargetX = targetX * odoRobot.countsPerInch - globalPositionUpdate.returnXCoordinate();
                    distanceToTargetY = targetY * odoRobot.countsPerInch - globalPositionUpdate.returnYCoordinate();
                    distance = Math.hypot(distanceToTargetX, distanceToTargetY);
                //gotoPosition(globalPositionUpdate, targetX, targetY, 0.3, 0.0, 1.0 );

                    moveToPoint(globalPositionUpdate, targetX, targetY, 0.8, 0.12, 00.0);
                    if (distanceToTargetY < 0 ) {
                        step = Step.robotTurn;
                    }
                    break;

                case robotTurn:
                    if (step == Step.robotTurn) {
                        double dAngle = 200;
                        odoRobot.robotTurn(tRight, dAngle, tAngle, fwdPwr);
                        break;
                    }




                    if (globalPositionUpdate.returnAngleDegree() < 0 ) {
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
            telemetry.addData("VortexStep", "%s", step);
            telemetry.addData("Distance ", distance / odoRobot.countsPerInch);
            telemetry.addData(" Time", "%s", currentTime);
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
    public void moveToPoint(OdometryCoordinatePosition ngpu, double targetX, double targetY, double robotPwr, double turnPwr, double desiredOrientation) {
        double fl, fr, bl, br;
        distanceToTargetX = targetX * robot.countsPerInch - ngpu.returnXCoordinate();
        distanceToTargetY = targetY * robot.countsPerInch - ngpu.returnYCoordinate();

        while (distanceToTargetY > 0) {
            distanceToTargetX = targetX * robot.countsPerInch - ngpu.returnXCoordinate();
            distanceToTargetY = targetY * robot.countsPerInch - ngpu.returnYCoordinate();
            double absoluteAngleToPoint = Math.atan2(distanceToTargetY, distanceToTargetX);

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

}