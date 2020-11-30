package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaSkyStone;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.TfodSkyStone;
import com.qualcomm.robotcore.util.ElapsedTime;


import java.util.ArrayList;


import HelperClass.Robot;
import HelperClass.RobotVision;

@Autonomous(name="Auto-Red", group="Linear Opmode")

public class RedAuto extends LinearOpMode {

    public RobotVision rVision;
    public String ringCountOnField = null;
    public String ringCountOnFieldLast = null;
    private boolean ringFoundConfirmed = false;

    public int autonomousStep = 1;
    public float maxSpeed = 0.5f;
    public float medSpeed = 0.25f;
    float offset;
    float angleOne;
    float angleTwo;
    double position = 0.5;
    float ringsShot = 0;


    Robot robot = new Robot();
    private ElapsedTime runtime = new ElapsedTime();
    private double currentTime = runtime.seconds();



    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap);
        rVision = new RobotVision();
        rVision.initVuforia(hardwareMap);
        rVision.initTfod(hardwareMap);
        robot.setupStartPosition();
        robot.startingAngle = 0;

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        rVision.activateTFOD();


        while (!opModeIsActive() && !isStopRequested()) {
            // Now get the number of rings on the field
            rVision.updateRingCount();
            ringCountOnField = rVision.getRingCount();
            ringCountOnFieldLast = ringCountOnField;

            // This code should move to a method in Robot Class
            // 24Nov
//            int correct = robot.arm.getCurrentPosition();
//            robot.arm.setPower(0);
//            robot.arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//            robot.claw.setPosition(-1);
//            robot.ringPush.setPosition(0);

            telemetry.addData("RingCount", "%s", ringCountOnField);
            telemetry.addData("Robot Angle", "%.1f", robot.getRobotAngle());
            telemetry.addData("FLMotor-1", "%d ", robot.FLMotor.getCurrentPosition());
            telemetry.addData("status", "waiting for a start command...");
            telemetry.addData("currentTime", "%s", currentTime);
            //telemetry.addData("arm position", "%s", correct);
            telemetry.update();
        }
//        waitForStart();

        angleOne = robot.getRobotAngle();
        while (opModeIsActive()) {

            switch (autonomousStep) {
                case 1:   // This is the start position
                     ringCountOnField = "Quad";
                    if (!ringFoundConfirmed&&currentTime<2) {
                        robot.movePower = medSpeed;
                        robot.moveTarget = 12.0f;
                        robot.moveRobot();
                        autonomousStep=300;
                    }
                    autonomousStep=300;
                    break;
                    case 200:
                    if (robot.moveStep == 3) {
                        //storedMineralposition = visionSystem.getmineralPostion();
                        if ((robot.period.seconds() - currentTime > 10)) {
                            //Not Found go on
                            autonomousStep = 300;
                        }
                        /*
                        if ((ringCountOnField == "single" || storedMineralposition == "quad") {
                            autonomousStep = 300;
                         */
                    }
                    break;
//strafes left
                case 300: // spin here

                   /* rVision.updateRingCount();
                    ringCountOnField=rVision.getRingCount();
                    ringCountOnFieldLast = ringCountOnField;

                    */
                    if (autonomousStep == 300) {
                        //prev. ringCountConfirmed
                        robot.movePower = -medSpeed;
                        robot.moveTarget = 15.0f;
                        robot.moveRobot();
                        autonomousStep = 301;
                    }
                    break;

                    /*
                    if (robot.moveStep == 3 || robot.moveStep == 0) {

                        robot.movePower = 0.4f;
                        robot.moveTarget = 19.0f;
                        robot.strafe();
                        autonomousStep = 301;
                    }

                     */
                //goes forward to case C
                case 301:
                    if (currentTime > 4) {
                        robot.movePower = maxSpeed;
                        robot.moveTarget = 96.0f;
                        robot.moveRobot();
                    }
                    autonomousStep = 400;
                    break;

//strafes right
                case 400:
                    if (ringCountOnField == "Quad" && currentTime > 9.4) {
                        telemetry.addData("quad ran", "%s", currentTime);
                        robot.movePower = maxSpeed;
                        robot.moveTarget = 15.0f;
                        robot.strafe();
                        if (currentTime > 12.86) {
                            autonomousStep = 999;
                        }
                    } else if (ringCountOnField == "Single" && currentTime > 8) {
                        telemetry.addData("single ran", "%s", currentTime);
                        robot.movePower = -maxSpeed;
                        robot.moveTarget = 15.0f;
                        robot.strafe();
                        autonomousStep = 1000;
                    } else if (ringCountOnField == "Null" && currentTime > 5) {
                        autonomousStep = 601;

                    }
                    break;

                //drops wobble goal
                case 999:
                    if (autonomousStep == 999) {
                        telemetry.addData("case 999", "%s", currentTime);
                        robot.arm.setTargetPosition(1);
                        robot.arm.setPower(-0.25);
                        //  autonomousStep = 400;
                        if (currentTime > 12.9) {
                            robot.claw.setPosition(1);
                            if (currentTime > 13) {
                                autonomousStep = 1000;
                            }
                        }
                    }
                    //brings arm up and goes forward
                case 1000:
                    if (autonomousStep == 1000) {
                        robot.arm.setTargetPosition(1);
                        robot.arm.setPower(0.25);
                        telemetry.addData("case 1000", "%s", currentTime);
                        if (currentTime > 13.3) {
                            robot.movePower = maxSpeed;
                            robot.moveTarget = 15.0f;
                            robot.moveRobot();
                            if (currentTime > 13.6) {
                                autonomousStep = 4000;
                            }
                        }
                        break;
                    }
                    //goes backward
                case 4000:
                    if (autonomousStep == 4000 && currentTime > 13.6) {
                        telemetry.addData("case 4000", "%s", currentTime);
                        robot.movePower = -maxSpeed;
                        robot.moveRobot();
                        robot.movePower = -maxSpeed;
                        robot.moveTarget = 15.0f;
                        robot.strafe();
                        autonomousStep = 4444;
                    }

                    break;
                case 4444:
                    if (autonomousStep == 4444 && currentTime > 16) {
                        robot.movePower = -maxSpeed;
                        robot.moveTarget = 15.0f;
                        robot.moveRobot();
                        if (autonomousStep == 4444 && currentTime > 18.6) {
                            autonomousStep = 4001;
                        }
                    }

                case 501:
                    if (autonomousStep == 501 && currentTime > 11) {
                        telemetry.addData("single ran", "%s", currentTime);
                        robot.movePower = -maxSpeed;
                        robot.moveTarget = 15.0f;
                        robot.strafe();
                        autonomousStep = 1000;

                    }
                    break;

                case 601:
                    if (autonomousStep == 601 && currentTime > 10) {
                        telemetry.addData("Null ran", "%s", currentTime);
                        robot.movePower = -maxSpeed;
                        robot.moveTarget = 18.0f;
                        robot.strafe();
                        autonomousStep = 1001;

                    }
                    break;
                case 1001:
                    if (autonomousStep == 1001 ) {

                        //&& currentTime > 15
                        telemetry.addData("case 1000", "%s", currentTime);
                       // robot.ringPush.setPosition(0);
                      //  wait(150);
                        runtime.reset();
                        currentTime=runtime.seconds();
                        autonomousStep = 4001;
                    }

                    break;

                case 4001:
                    if (autonomousStep == 4001) {
                        robot.shooting1.setPower(-0.73);
                        robot.shooting2.setPower(-0.73);
                        robot.ringPush.setPosition(0);
                        while (autonomousStep == 4001 && currentTime > 18.5) {
                            robot.ringPush.setPosition(1);
                            if(autonomousStep == 4001 &&currentTime>19) {
                                ringsShot++;
                            }
                            if (ringsShot >= 2) {
                                robot.intake.setPower(-3);
                                robot.ringPush.setPosition(1);
                                autonomousStep = 1001;

                            }
                            autonomousStep = 1001;

                        }

                        break;

                    }
                    break;


            }

            if (currentTime < 30) {

                currentTime = runtime.seconds();
            } else {
                break;
            }

            if (ringCountOnField == null) {
                rVision.updateRingCount();
                ringCountOnField = rVision.getRingCount();
                // if(ringCountOnField!=null);
            } else {
                ringFoundConfirmed = true;
            }
//                    robot.moveUpdate();
                      telemetry.addData("OpModeActive", "Running");
//                    telemetry.addData("ringpush", "%d ", robot.ringPush.getPosition());
                      telemetry.addData("currentTime", "%s", currentTime);
                      telemetry.addData("Autonomous Step", "%d ", autonomousStep);
//                    telemetry.addData("Runtime", "%s", runtime);
                      telemetry.addData("ringFoundConfirm", "%s", ringFoundConfirmed);
                      telemetry.addData("RingCount", "%s", ringCountOnField);
//                     telemetry.addData("RingCountLast", "%s", ringCountOnFieldLast);
//                    telemetry.addData("Robot Angle", "%.1f", robot.getRobotAngle());
//                    telemetry.addData("FLMotor-2", "%d ", robot.FLMotor.getCurrentPosition());
                         telemetry.update();
                      //  telemetry.addData("ringsShot", "%s", ringsShot);

            }
        }
    }
//a = 5ft, b = 7ft, c = 9 ft
