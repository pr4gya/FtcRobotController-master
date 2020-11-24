package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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

@Autonomous(name="Angle Adjustment", group="Linear Opmode")
@Disabled
public class armtest extends LinearOpMode {
    public int autonomousStep = 1;
    public float maxSpeed = 0.5f;
    public float medSpeed = 0.25f;
    float offset;
    float angleOne;
    float angleTwo;


    Robot robot = new Robot();
    private ElapsedTime runtime = new ElapsedTime();
    private double currentTime = runtime.seconds();


    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap);

        robot.startingAngle = 0;

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        int correct = robot.arm.getCurrentPosition();


        while (!opModeIsActive() && !isStopRequested()) {
            currentTime = 0;
            telemetry.addData("Robot Angle", "%.1f", robot.getRobotAngle());
            telemetry.addData("FLMotor-1", "%d ", robot.FLMotor.getCurrentPosition());
            telemetry.addData("status", "waiting for a start command...");
            telemetry.addData("currentTime", "%s", currentTime);
            telemetry.update();
           // int correct = robot.arm.getCurrentPosition();
            robot.arm.setTargetPosition(correct);
            robot.arm.setPower(0);
            robot.arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.claw.setPosition(-1);
            autonomousStep=400;
            angleOne = robot.getRobotAngle();
        }

        runtime.reset();
        currentTime = runtime.seconds();
        waitForStart();
        runtime.reset();
        currentTime = runtime.seconds();
        float angleOne = robot.getRobotAngle();
        angleOne = (float) (angleOne / 180 * Math.PI);
        autonomousStep=400;
        while (opModeIsActive()) {
            //autonomousStep=1;
            //currentTime=14;
            // Autonomous state machine is executed here
            //ringFoundConfirmed = false;

            switch (autonomousStep) {
                case 1:   // This is the start position
                    // currentTime=0;
                    //robot.moveRobot();
                    if (autonomousStep == 1) {
                        //currentTime = robot.period.seconds();
                        robot.movePower = medSpeed;
                        robot.moveTarget = 12.0f;
                        robot.moveRobot();
                        //currentTime = robot.period.seconds()
                        autonomousStep = 300;
                        //ringFoundConfirmed=true;
                    } else {
                        autonomousStep = 300;
                    }
                    break;
                case 300:
                    if (autonomousStep == 300 && currentTime > 3) {
                        robot.arm.setTargetPosition(1);
                        robot.arm.setPower(-0.25);
                        //  autonomousStep = 400;
                        if (currentTime > 4) {
                            robot.claw.setPosition(1);
                        }
                    }


                case 400:
                    // current angle
                    if(currentTime>3&&autonomousStep==400) {
                        angleTwo = robot.getRobotAngle();
                        angleTwo = (float) (angleTwo / 180 * Math.PI);
                        while (angleTwo - angleOne < 0) {
                            robot.FLMotor.setPower(0.3);
                            robot.BLMotor.setPower(0.3);
                            robot.FRMotor.setPower(-0.3);
                            robot.BRMotor.setPower(-0.3);
                            angleTwo = robot.getRobotAngle();
                            angleTwo = (float) (angleTwo / 180 * Math.PI);
                        }
                        autonomousStep = 420;

                        while(angleTwo - angleOne > 0) {
                            robot.FLMotor.setPower(-0.3);
                            robot.BLMotor.setPower(-0.3);
                            robot.FRMotor.setPower(0.3);
                            robot.BRMotor.setPower(0.3);
                            angleTwo = robot.getRobotAngle();
                            angleTwo = (float) (angleTwo / 180 * Math.PI);
                        }
                        autonomousStep = 421;
                    }
                    break;


//error of accuracy uses -2
                //constantly checks if it has moved
                case 420:
                    angleTwo = robot.getRobotAngle();
                    angleTwo = (float) (angleTwo/180*Math.PI);
                    if (angleOne - angleTwo  >= -2) {
                        robot.FLMotor.setPower(0);
                        robot.BLMotor.setPower(0);
                        robot.FRMotor.setPower(0);
                        robot.BRMotor.setPower(0);
                        robot.FLMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        robot.BLMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        robot.FRMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        robot.BRMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        robot.FLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        robot.BLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        robot.FRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        robot.BRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    }

                    else {
                        autonomousStep=400;
                    }
                    break;


                case 421:
                    angleTwo = robot.getRobotAngle();
                    angleTwo = (float) (angleTwo/180*Math.PI);
                    if (angleOne - angleTwo <= 2) {
                        robot.FLMotor.setPower(0);
                        robot.BLMotor.setPower(0);
                        robot.FRMotor.setPower(0);
                        robot.BRMotor.setPower(0);
                        robot.FLMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        robot.BLMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        robot.FRMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        robot.BRMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        robot.FLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        robot.BLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        robot.FRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        robot.BRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                                           }
                    else{
                        autonomousStep=400;
                }
                    break;


                case 999:
                    if (autonomousStep == 999) {
                        telemetry.addData("case 999", "%s", currentTime);
                        angleTwo = robot.getRobotAngle();
                        if (angleOne > angleTwo)
                            offset = angleOne - angleTwo;
                        else {
                            offset = angleTwo - angleOne;
                        }
                        if (currentTime > 14) {
                            autonomousStep = 1000;
                        }
                    }

                case 4001:
                    if (autonomousStep == 4001 && currentTime > 15) {
                        //   telemetry.addData("case 1000", "%s", currentTime);
                        robot.shooting1.setPower(-0.65);
                        robot.shooting2.setPower(-0.65);
                        while (autonomousStep == 4001 && currentTime > 16.8) {
                            robot.ringPush.setPosition(0);
                            robot.ringPush.setPosition(0.2);
                            robot.intake.setPower(-1);
                            autonomousStep = 1001;
                        }

                        break;

                    }
                    break;

            }
            if (currentTime < 30) {

                currentTime = runtime.seconds();
            }

            else{
                break;
            }

            robot.moveUpdate();
            telemetry.addData("OpModeActive", "Running");
            telemetry.addData("currentTime", "%s", currentTime);
            telemetry.addData("Autonomous Step", "%d ", autonomousStep);
            telemetry.addData("Runtime", "%s", runtime);
            //telemetry.addData("RingCountLast", "%s", ringCountOnFieldLast);
            telemetry.addData("Robot Angle", "%.1f", robot.getRobotAngle());
            telemetry.addData("FLMotor-2", "%d ", robot.FLMotor.getCurrentPosition());
            telemetry.update();

        }

    }
}

//a = 5ft, b = 7ft, c = 9 ft
