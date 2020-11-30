//package org.firstinspires.ftc.teamcode;
//
//import java.lang.System.*;
//import android.os.SystemClock;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.util.ElapsedTime;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import java.util.List;
//import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
//import org.firstinspires.ftc.robotcore.external.navigation.VuforiaSkyStone;
//import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
//import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
//import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
//import org.firstinspires.ftc.robotcore.external.tfod.TfodSkyStone;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import HelperClass.Robot;
//import HelperClass.RobotVision;
//
//@Autonomous(name="Vortex Auto-Red", group="Linear Opmode")
//
//public class VortexRedAuto extends LinearOpMode {
//
//    public RobotVision rVision;
//    public String ringCountOnField = null;
//    public String ringCountOnFieldLast = null;
//
//    private boolean ringFoundConfirmed = false;
//    private String programState = null;
//
//    public long programInitTime = 0;//time the program init
//    public long programStartTime = 0;//time the program starts
//    public long programCurrentTime = 0;//time the current time in the program
//    public long stateStartTime = 0;//time the program state starts
//    public long stateEndTime = 0;//time the program state end
//
//
//
//    public float maxSpeed = 0.5f;
//    public float medSpeed = 0.25f;
//
//    float offset;
//    float angleOne;
//    float angleTwo;
//    double position = 0.5;
//    float ringsShot = 0;
//
//    public enum AutonomousStep {
//        notReadyYet, startTheGame, moveForward, unlockIntack, lockArmPosition, moveStrafe,
//        dropWobble, gotoShotLine, shootToGoal, shootPowerShot, somethingWrong, endAuto, forTesting
//    }
//
//    public AutonomousStep autonomousStep = AutonomousStep.notReadyYet;
//    // For testing use this
//    //public AutonomousStep autonomousStep = AutonomousStep.forTesting;
//
//    Robot robot = new Robot();
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//
//
//        programInitTime = SystemClock.uptimeMillis();
//        robot.init(hardwareMap);
//        rVision = new RobotVision();
//        rVision.initVuforia(hardwareMap);
//        rVision.initTfod(hardwareMap);
//        robot.setupStartPosition();
//        Robot.startingAngle = 0;
//
//        telemetry.addData("Status", "Initialized");
//        telemetry.update();
//
//
//        rVision.activateTFOD();
//
//
//        while (!opModeIsActive() && !isStopRequested()) {
//            // Now get the number of rings on the field
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
//        }
//
//        waitForStart();
//
//        programStartTime = SystemClock.uptimeMillis();
//
//
//        angleOne = robot.getRobotAngle();
//
//        while (opModeIsActive()) {
//
//            switch (autonomousStep) {
//
//                // This is to check that ring has been detected before it starts moving
//                case notReadyYet:
//                    programState = "NotReadyYet";
//                    autonomousStep = AutonomousStep.startTheGame;
//                    // Next 2 lines enable Only for testing with OD
//                    //ringCountOnField = "Quad";
//                    //ringFoundConfirmed = true;
//                    break;
//
//                case startTheGame:
//                    programState = "startTheGame";
//                    stateStartTime = SystemClock.uptimeMillis();
//
//                    if (ringFoundConfirmed) {
//                        if (ringCountOnField == "Quad") {
//                            robot.movePower = medSpeed;
//                            robot.ringCount = 4;
//                            robot.moveRobotN(0.25f,4, 96.0f, 20.0f );
//                        } else if (ringCountOnField == "Single") {
//                            robot.movePower = medSpeed;
//                            robot.ringCount = 1;
//                            robot.moveRobotN(0.25f,1, 72.0f, 10.0f );
//                        } else {
//                            robot.movePower = medSpeed;
//                            robot.ringCount = 0;
//                            robot.moveRobotN(0.25f,0, 48.0f, 20.0f );
//                        }
//
//                        autonomousStep = AutonomousStep.moveForward;
//
//                    }
//                    break;
//
//                case moveForward:
//                    programState = "moveForward";
//
//                    if (robot.moveStep == 19 ) {
//                        autonomousStep = AutonomousStep.dropWobble;
//                        stateEndTime = SystemClock.uptimeMillis();
//                    } else {
//                        robot.moveRobot();
//                        autonomousStep = AutonomousStep.moveForward;
//                    }
//                    break;
//
//                case dropWobble:
//                    programState = "dropWobble";
//
//                    // back to start line
//                    break;
//
//                case gotoShotLine:
//
//                    // Goto to shot line and adjust angle
//                    break;
//
//                case shootToGoal:
//
//                    // 2 Rings ShootToGoal
//                    break;
//
//                case shootPowerShot:
//
//                    // Adjust Angle for powerShot
//                    // Shoot PowerShot
//                    break;
//
//                case somethingWrong:
//                    programState = "somethingWrong";
//                    autonomousStep = AutonomousStep.somethingWrong;
//                    break;
//
//                case endAuto:   // End Auto here
//                    //autonomousStep = AutonomousStep.moveForward;
//                    break;
//
//            }
//
//            programCurrentTime = SystemClock.uptimeMillis();
//            if (ringCountOnField == null) {
//
//                if (programCurrentTime - programStartTime >= 4000) {
//                    ringCountOnField = "Zero";
//                    autonomousStep = AutonomousStep.startTheGame;
//                } else {
//                    rVision.updateRingCount();
//                    ringCountOnField = rVision.getRingCount();
//                    autonomousStep = AutonomousStep.notReadyYet;
//                }
//            } else {
//                ringFoundConfirmed = true;
//            }
//            robot.moveUpdate();
//            telemetry.addData("ProgramTime", "%d",(programCurrentTime - programStartTime));
//            telemetry.addData("ProgramState", "%s", programState);
//            telemetry.addData("RobotProgState", "%d", robot.moveStep);
//            telemetry.addData("FLMotor", "%d ", Math.abs(robot.FLMotor.getCurrentPosition()));
//            telemetry.addData("target Position", "%d ", 6*90);
//            telemetry.addData("Robot Angle", "%.1f", angleOne);
//            telemetry.addData("Autonomous Step", "%s ", autonomousStep);
//            telemetry.addData("ringFoundConfirm", "%s", ringFoundConfirmed);
//            telemetry.addData("RingCount", "%s", ringCountOnField);
//            telemetry.update();
//
//
//        }
//    }
//}
////a = 5ft, b = 7ft, c = 9 ft
