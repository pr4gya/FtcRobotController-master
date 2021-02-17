package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import HelperClass.Robot;
import Hardware.DriveTrainNew;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@TeleOp(name="TeleOp", group="Pushbot")
public class Vortex14969Teleop extends LinearOpMode {

    Robot robot = new Robot();
    private DriveTrainNew myDriveTrain;
    private ElapsedTime runtime = new ElapsedTime();
    private double currentTime  = runtime.seconds();

    double clawPosition = robot.CLAW_HOME;
    final double CLAW_SPEED = 0.01;
    double front_distance;
    double red_side_wall_distance;

    float offset;
    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        //now we can initialize the myDriveTrai n
        myDriveTrain = new DriveTrainNew(robot.FLMotor, robot.FRMotor, robot.BLMotor, robot.BRMotor);
        telemetry.addData("Status", "Initialized 1");
        telemetry.update();

        while (!opModeIsActive() && !isStopRequested()) {
            int correct = robot.arm.getCurrentPosition();
            robot.claw.setPosition(-1);
            robot.ringPush.setPosition(1); // Ring Push mechanism is set to start position
            telemetry.addData("Rotation", "%f ", robot.getRobotAngle());
            telemetry.update();

            telemetry.addData("curr pos", "%s",robot.arm.getCurrentPosition());
            telemetry.addData("correct", "%s", correct);
        }

        while (opModeIsActive()) {
            runtime.reset();
            currentTime = runtime.seconds();

            // Convert joysticks to desired motion
            // myDriveTrain.ApplyMovement(
            //         gamepad1.left_stick_x, gamepad1.left_stick_y,
            //         gamepad1.right_stick_x, gamepad1.right_stick_y);


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


            //when button is held, intake spins. When button released, then intake stops all motion
            float reverseIntake = gamepad2.left_trigger;
            boolean intakePower = gamepad2.left_bumper;
            float claw = gamepad2.right_trigger;
            float arm = gamepad2.left_stick_y;

            //sets intake and reverse intake values
            if (intakePower) {
                robot.intake.setPower(-1);
            } else if (reverseIntake > 0) {
                robot.intake.setPower(1);
            } else {
                robot.intake.setPower(0);
            }

            //when button pressed, shooting motors are turned on
            boolean ifShoot = gamepad2.right_bumper;
            //power of the shooting motors can be controlled using dpad right and left

//            if (gamepad1.a) {
//                clawPosition += CLAW_SPEED;
//            } else if (gamepad1.y) {
//                clawPosition -= CLAW_SPEED;
//            }
//
//
//            clawPosition = Range.clip(clawPosition, robot.CLAW_MIN, robot.CLAW_MAX);
//            robot.claw.setPosition(clawPosition);

            if (gamepad2.right_trigger > 0) {
                robot.claw.setPosition(1);
            } else {
                robot.claw.setPosition(-1);
            }

            if (gamepad2.a) {
                robot.arm.setTargetPosition(1);
                robot.arm.setPower(0.25);
            }
            else if (gamepad2.b){
                robot.arm.setTargetPosition(-1);
                robot.arm.setPower(-0.25);
            }
            else{
                robot.arm.setTargetPosition(0);
                robot.arm.setPower(0);
                robot.arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }

            //if (gamepad2.dpad_right) {
            //    shootPower = 0.8;
            //}
            //if (gamepad2.dpad_left) {
            //    shootPower = -0.5;
            //}


            if (ifShoot) {
                //robot.shooting1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                //robot.shooting2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                shootPower = -0.6;
                robot.shooting1.setPower(shootPower);
                robot.shooting2.setPower(shootPower);
            }


            if (ifShoot == false) {
                robot.shooting1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                robot.shooting2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                robot.shooting1.setPower(0);
                robot.shooting2.setPower(0);
            }


//            if(gamepad1.b){
//                robot.adjustAngle();
//            }


            boolean intakeHolder = gamepad2.x;
            if (gamepad2.x) {
                robot.intakeHolder.setPosition(0.2);
            } else {
                robot.intakeHolder.setPosition(0.5);
            }

            boolean ringPush = gamepad2.y;
            if (gamepad2.y) {
                robot.ringPush.setPosition(-1);
            } else {
                robot.ringPush.setPosition(1);
            }



            //robot.intakeHolder.setDirection(2);
            // End of Code for Motor Movement

            front_distance = robot.frontRange.getDistance(DistanceUnit.INCH);

            red_side_wall_distance = robot.redWallRange.getDistance(DistanceUnit.INCH);


            telemetry.addData("Claw Pos: ", "%f", clawPosition);
            telemetry.addData("curr pos", "%s",robot.arm.getCurrentPosition());
            telemetry.addData("Robot Angle: ", "%f", robot.getRobotAngle());
            telemetry.addData("Status", "Running");
            telemetry.addData("Front range", String.format("%.01f cm", front_distance));
            telemetry.addData("Red Wall range", String.format("%.01f cm", red_side_wall_distance));
            telemetry.addData("Rotation", "%f ", robot.getRobotAngle());
            telemetry.addData("Servo Position: ", "%f", robot.ringPush.getPosition());
            telemetry.addData("Reverse intake power", "%f", reverseIntake);
            telemetry.update();


            }
            //end while(OpmodeIsActive);

        }//end public void runopmode();
    }