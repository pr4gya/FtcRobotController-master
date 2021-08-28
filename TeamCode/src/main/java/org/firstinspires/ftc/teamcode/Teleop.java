package org.firstinspires.ftc.teamcode;
import android.os.SystemClock;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import HelperClass.Robot;
import Hardware.DriveTrainNew;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@TeleOp(name="Optimized TeleOp", group="Pushbot")
public class Teleop extends LinearOpMode {

    Robot robot = new Robot();
    private DriveTrainNew myDriveTrain;
    private ElapsedTime runtime = new ElapsedTime();
    private double currentTime  = runtime.seconds();

    double front_distance;
    double red_side_wall_distance;
    double shootPower = 0;
    public double angleOne;
    int y = 0;

    double p = 0.4;
    double a = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        //now we can initialize myDriveTrain
        myDriveTrain = new DriveTrainNew(robot.FLMotor, robot.FRMotor, robot.BLMotor, robot.BRMotor);
        telemetry.addData("Status", "Initialized ");
        telemetry.update();

        while (!opModeIsActive() && !isStopRequested()) {
            robot.claw.setPosition(-1);
            robot.intakeHolder.setPosition(-0.5);
            robot.ringPush.setPosition(1);
            angleOne = robot.getRobotAngle();
            telemetry.addData("Angle", "%f ", robot.getRobotAngle());
            telemetry.addData("Robot Angle", "%.1f", angleOne);
            telemetry.update();

        }

        while (opModeIsActive()) {
            runtime.reset();
            currentTime = runtime.seconds();

            //drivetrain motors
            double move_y_axis = -gamepad1.left_stick_y;      // remember, this is reversed
            double move_x_axis = gamepad1.left_stick_x * 1.5; // counteract imperfect strafing
            double pivot_turn = gamepad1.right_stick_x;

            double fl_power = move_y_axis + move_x_axis + pivot_turn;
            double bl_power = move_y_axis - move_x_axis + pivot_turn;
            double fr_power = move_y_axis - move_x_axis - pivot_turn;
            double br_power = move_y_axis + move_x_axis - pivot_turn;

            //now we can set the powers
            robot.FLMotor.setPower(fl_power);
            robot.BLMotor.setPower(bl_power);
            robot.FRMotor.setPower(fr_power);
            robot.BRMotor.setPower(br_power);


         //other motors
            if (gamepad2.a || gamepad1.a) { //arm - forward and reversed
                robot.arm.setPower(1);
            }
            else if (gamepad2.b || gamepad2.b){
                robot.arm.setPower(-1);
            }
            else{
                robot.arm.setPower(0);
                robot.arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }
            if (gamepad2.left_trigger<0 || gamepad1.left_trigger<0) { //intake - forward and reversed
                robot.intake.setPower(-1);
            } else if (gamepad2.left_trigger > 0 || gamepad1.left_trigger>0) {
                robot.intake.setPower(1);
            } else {
                robot.intake.setPower(0);
            }

        //servos:
            if (gamepad2.x) { //intake holder
                robot.intakeHolder.setPosition(1);
            } else {
                robot.intakeHolder.setPosition(-0.5);
            }
            if (gamepad2.y) { //ring push
                robot.ringPush.setPosition(-1);
            } else {
                robot.ringPush.setPosition(1);
            }
            if (gamepad2.right_trigger > 0) { //claw
                robot.claw.setPosition(1);
            } else {
                robot.claw.setPosition(-1);
            }

        //autonomous optimizations
            //high goal shooting



            if (gamepad1.a){
                robot.shooting1.setPower(-p);
                robot.shooting2.setPower(-0.2);
                SystemClock.sleep(500);
                for (int i =1; i<=3; i++){
                    robot.ringPush.setPosition(-1);
                    SystemClock.sleep(300);
                    robot.ringPush.setPosition(1);
                    SystemClock.sleep(300);
                }
                robot.shooting1.setPower(0);
                robot.shooting2.setPower(0);
            }

            if (gamepad1.b) {
                if(y<=3){
                p=p+0.05;
                y++;
                }
            }



//            front_distance = robot.frontRange.getDistance(DistanceUnit.INCH);
//            red_side_wall_distance = robot.redWallRange.getDistance(DistanceUnit.INCH);
            telemetry.addData("pls", "%f", p);
            telemetry.addData("Status", "Running");
            telemetry.addData("Robot Angle: ", "%f", robot.getRobotAngle());
//            telemetry.addData("Front range", String.format("%.01f cm", front_distance));
//            telemetry.addData("Red Wall range", String.format("%.01f cm", red_side_wall_distance));
            telemetry.addData("Servo Position: ", "%f", robot.ringPush.getPosition());
            telemetry.update();

        }
        //end while(OpmodeIsActive);

    }//end public void runopmode();
}