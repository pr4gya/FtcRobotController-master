package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.robotcore.util.ElapsedTime;
import android.os.SystemClock;

import HelperClass.OdometryRobot;
import HelperClass.RobotVision;

import com.qualcomm.robotcore.hardware.DcMotor;



@Autonomous(name="OdometryAuto", group="Linear Opmode")
public class VortexOdometryAuto extends LinearOpMode {

    //calls helper classes
    public RobotVision rVision;
    // public Robot robot;
    OdometryRobot robot = new OdometryRobot();

    //ring count vars
    public String ringCountOnField = null;
    public String ringCountOnFieldLast = null;
    private boolean ringFoundConfirmed = false;


    private ElapsedTime runtime = new ElapsedTime();
    private double currentTime = runtime.seconds();
    private double startTime;


    //enum vars defined
    private enum Step {confirmation, startMv, checkRings, mvBack, mvForward, mvStrafe, dropWobble, shootRing,
        launchLine, quad, single, none, angleAdj, park
    }

    private Step step = Step.confirmation;


    @Override
    public void runOpMode() throws InterruptedException {
        runtime.reset();
        //angleOne = robot.getRobotAngle();
        startTime = runtime.seconds();
        robot.init(hardwareMap);

        /* Enable for vision
        rVision = new RobotVision();
        rVision.initVuforia(hardwareMap);
        rVision.initTfod(hardwareMap);
        rVision.activateTFOD();
        */

        while (!opModeIsActive() && !isStopRequested()) {
            telemetry.addData("Initialization Time", "%s", startTime);
            telemetry.addData("Robot Angle", "%.1f", robot.getRobotAngle());
            telemetry.addData("Ring Count Confirmed", "%s", ringFoundConfirmed);
            telemetry.addData("Ring Count", "%s", ringCountOnField);
            telemetry.addData("Ring Count", "%s", ringCountOnFieldLast);
            telemetry.addData("Claw Position", "%s", robot.claw.getPosition());
            telemetry.update();

            /* Enable for vision
            rVision.updateRingCount();
            ringCountOnFieldLast = rVision.getRingCount();
            rVision.updateRingCount();
            ringCountOnField = rVision.getRingCount();

            int armpos = robot.arm.getCurrentPosition();
            robot.arm.setTargetPosition(armpos);
            robot.arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.claw.setPosition(0);
            */
        }
        waitForStart();

        runtime.reset();
        startTime = runtime.seconds();

        while(opModeIsActive()) {

            switch (step) {
                case confirmation:
                        // Check to confirm that ring has been detected
                    break;

                case startMv:
                    // Goto Position A on the filed to deliver wobble goal


                case none:
                    break;

            }

            //robot.moveUpdate();

            telemetry.addData("Current Time", "%s", currentTime);
            telemetry.addData("VortexStep", "%s", step);
            telemetry.addData("Robot Angle", "%.1f", robot.getRobotAngle());
            telemetry.addData("Claw Position-2", "%s", robot.claw.getPosition());
            telemetry.addData("Ring Push", "%s", robot.ringPush.getPosition());
            telemetry.update();

        }

    }
}
