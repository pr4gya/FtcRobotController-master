package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import HelperClass.OdometryRobot;
import HelperClass.Robot;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;


@TeleOp(name="Dist Test", group="Pushbot")
//@Disabled
public class distTest extends LinearOpMode {

    OdometryRobot robot = new OdometryRobot();;
    //Robot robot = new Robot();

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);


        while (!opModeIsActive() && !isStopRequested()) {
            telemetry.addData("Status", "Initialized 1");
            telemetry.addData("rangeFront", robot.rangeFront);
            telemetry.addData("rangeLeft", robot.rangeLeft);
            telemetry.addData("rangeRight", robot.rangeRight);
            telemetry.update();

            if (gamepad1.x) {
                robot.move(1.0, 2.0);
            }

            while (opModeIsActive()) {

                robot.goalShootAlign(0.45);

                telemetry.addData("Status", "Running");
                telemetry.addData("rangeFront", robot.rangeFront);
                telemetry.addData("rangeLeft", robot.rangeLeft);
                telemetry.addData("rangeRight", robot.rangeRight);
                telemetry.update();
            }
        }
    }


    public void goalShotAlign(double speed) {
        double distanceFront = 0;
        distanceFront = robot.rangeFront.getDistance(DistanceUnit.CM);

        robot.FLMotor.setPower(speed);
        robot.FRMotor.setPower(speed);
        robot.BLMotor.setPower(speed);
        robot.BRMotor.setPower(speed);
        while (distanceFront > 5) {
            distanceFront = robot.rangeFront.getDistance(DistanceUnit.CM);
        }
        robot.FLMotor.setPower(0);
        robot.FRMotor.setPower(0);
        robot.BLMotor.setPower(0);
        robot.BRMotor.setPower(0);
    }
}
