package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.robotcore.util.ElapsedTime;
import android.os.SystemClock;

import HelperClass.Robot;
import HelperClass.RobotVision;

@Autonomous(name="Red Autonomous", group="Linear Opmode")
public class Vortex14969RedAutonomous extends LinearOpMode {

    //calls helper classes
    public RobotVision rVision;
 //   public Robot robot;
    Robot robot = new Robot();

    //ring count vars
    public String ringCountOnField = null;
    public String ringCountOnFieldLast = null;
    private boolean ringFoundConfirmed = false;

    //powers
    public float maxSpeed = 0.5f;
    public float medSpeed = 0.5f;

    public float sideDist = 0.0f;
    public float fwdDist = 0.0f;
    public float speed = 0.0f;

    //launchline vars
    public float straightLD = 0.0f;
    public float sideLD = 0.0f;
    public float sidespeedL = 0.0f;
    public float straightspeed = 0.0f;

    //time vars
    public long tInit;    //init time
    public long tStart;  //start time in ms
    public long tsStart; //start time in s
    public long tElapsed; //elapsed time
    public float tCurrent; //takes current time value
    public long tEnd;     //end time
    private ElapsedTime runtime = new ElapsedTime();
    private double currentTime = runtime.seconds();
    private double startTime;

    //angle
   // float angleOne = 0;

    //enum vars defined
    public enum Step {confirmation, startMv, checkRings, mvBack, mvForward, mvStrafe, dropWobble, shootRing,
        launchLine, quad, single, none, angleAdj
    }
    public Step step = Step.confirmation;


    @Override
    public void runOpMode() throws InterruptedException {

        tInit = SystemClock.uptimeMillis();
      //  angleOne = robot.getRobotAngle();

        robot.init(hardwareMap);
        rVision = new RobotVision();
        rVision.initVuforia(hardwareMap);
        rVision.initTfod(hardwareMap);
        rVision.activateTFOD();


        while(!opModeIsActive()&&!isStopRequested()){
            telemetry.addData("Initialization Time", "%s", tInit);
            telemetry.addData("Robot Angle", "%.1f", robot.getRobotAngle());
            telemetry.addData("Start Time", "%s", tStart);
            telemetry.addData("Ring Count Confirmed", "%s", ringFoundConfirmed);
            telemetry.addData("Ring Count", "%s", ringCountOnField);
            telemetry.addData("Ring Count", "%s", ringCountOnFieldLast);
            telemetry.addData("Claw Position", "%s", robot.claw.getPosition());
            telemetry.update();
            rVision.updateRingCount();
            ringCountOnFieldLast = rVision.getRingCount();
            rVision.updateRingCount();
            ringCountOnField = rVision.getRingCount();
            int armpos = robot.arm.getCurrentPosition();
            robot.arm.setTargetPosition(armpos);
            robot.arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.claw.setPosition(0);
            //robot.ringPush.setPosition(0);
        }
        waitForStart();
        //

        runtime.reset();
        startTime = runtime.seconds();
        tStart = SystemClock.uptimeMillis();
        while(opModeIsActive()) {
           // step = Step.shootRing;

            switch (step) {
                case confirmation:
                    if (ringCountOnField == ringCountOnFieldLast && ringFoundConfirmed) {
                        //step = Step.mvStrafe;
                        step = Step.startMv;
                    }
                    break;

                case startMv:
                    if (ringCountOnField == "Zero") {
                        fwdDist = 38;
                        speed = medSpeed;
                        sideDist = 27.0f;
                        robot.mvDestination(fwdDist, speed, sideDist);
                    }

                    else if(ringCountOnField=="Single") {
                        fwdDist = 50;
                        speed = medSpeed;
                        sideDist = 10.0f;
                        robot.mvDestination(fwdDist, speed, sideDist);
                    }

                    else if (ringCountOnField=="Quad"){
                        fwdDist = 70;
                        speed = medSpeed;
                        sideDist = 25.0f;
                        robot.mvDestination(fwdDist, speed, sideDist);
                    }
                    step = Step.dropWobble;
                    break;

                case dropWobble:
                    step = Step.launchLine;
                    break;

                   case launchLine:
                   if (ringCountOnField=="Zero"){
                      straightLD = 0.0f;
                      straightspeed = 0.0f;
                      sideLD = 15.0f;
                      sidespeedL = -0.5f;
                      robot.mvlaunchLine(straightLD, straightspeed, sideLD, sidespeedL);
                  }
                  else if (ringCountOnField=="Single"){
                    straightLD = 0.0f;
                    straightspeed = 0.0f;
                    sideLD = 15.0f;
                    sidespeedL = -0.5f;
                    robot.mvlaunchLine(straightLD, straightspeed, sideLD, sidespeedL);
                  }
                  else if (ringCountOnField=="Quad"){
                    straightLD = 28.5f;
                    straightspeed = -0.75f;
                    sideLD = 15.0f;
                    sidespeedL = -0.5f;
                    robot.mvlaunchLine(straightLD, straightspeed, sideLD, sidespeedL);

                  }
                step = Step.angleAdj;
                break;

                case angleAdj:
                    robot.adjustAngle();
                    step = Step.shootRing;
                    break;

                case shootRing:
                break;


                case none:
                    break;

            }

        if (currentTime < 30) {
            currentTime = runtime.seconds();
        } else {
            break;
        }

        currentTime = runtime.seconds();
        if (ringCountOnField == null) {
            rVision.updateRingCount();
            ringCountOnField = rVision.getRingCount();
            if (currentTime > 3.0) {
                ringCountOnField = "Zero";
                ringCountOnFieldLast = ringCountOnField;
                ringFoundConfirmed = true;

            } else {
                rVision.updateRingCount();
                ringCountOnField = rVision.getRingCount();
            }
        } else {
                ringFoundConfirmed = true;
        }
        robot.moveUpdate();
        telemetry.addData("OpModeActive", "Running");
        telemetry.addData("Current Time", "%s", currentTime);
        telemetry.addData("VortexStep", "%s", step);
        telemetry.addData("RobotStep", "%s", robot.autostep);
        telemetry.addData("RingCount", "%s", ringCountOnField);
        telemetry.addData("Robot Angle", "%.1f", robot.getRobotAngle());
        telemetry.addData("FLMotor-2", "%d ", robot.FLMotor.getCurrentPosition());
        telemetry.addData("Claw Position-2", "%s", robot.claw.getPosition());
        telemetry.addData("Ring Push", "%s", robot.ringPush.getPosition());
        telemetry.update();

        }
    }
}
