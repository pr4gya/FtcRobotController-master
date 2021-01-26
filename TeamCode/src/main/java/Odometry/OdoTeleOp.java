package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import HelperClass.OdoRobot;
import Hardware.DriveTrainNew;
import InTakeHelper.ControllerB;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@TeleOp(name="Odometry TeleOp", group="Pushbot")
public class OdoTeleOp extends LinearOpMode {

    OdoRobot robot = new OdoRobot();
    private DriveTrainNew myDriveTrain;
    private ElapsedTime runtime = new ElapsedTime();
    private double currentTime  = runtime.seconds();

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
//            int correct = robot.arm.getCurrentPosition();
//            robot.claw.setPosition(0);
//            telemetry.addData("Rotation", "%f ", robot.getRobotAngle());
//            telemetry.update();
//
//            telemetry.addData("curr pos", "%s",robot.arm.getCurrentPosition());
//            telemetry.addData("correct", "%s", correct);
            telemetry.addLine("initialized");
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
            //now we can set the powers
            robot.FLMotor.setPower(fl_power);
            robot.BLMotor.setPower(bl_power);
            robot.FRMotor.setPower(fr_power);
            robot.BRMotor.setPower(br_power);


            // End of Code for Motor Movement

//            front_distance = robot.frontRange.getDistance(DistanceUnit.INCH);
//
//            red_side_wall_distance = robot.redWallRange.getDistance(DistanceUnit.INCH);
//

//
//            telemetry.addData("Robot Angle: ", "%f", robot.getRobotAngle());
//            telemetry.addData("Rotation", "%f ", robot.getRobotAngle());
            telemetry.addLine("running");
            telemetry.update();


        }
        //end while(OpmodeIsActive);

    }//end public void runopmode();
}