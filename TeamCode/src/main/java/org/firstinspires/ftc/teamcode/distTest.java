package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import HelperClass.OdometryRobot;
import Odometry.Main.Odometry;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


@TeleOp(name="Dist Test", group="Pushbot")
//@Disabled
public class distTest extends LinearOpMode {

OdometryRobot robot =  new OdometryRobot();

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);


        while(!opModeIsActive() && !isStopRequested()){
            telemetry.addData("Status", "Initialized 1");
            telemetry.addData("rangeFront", robot.rangeFront);
            telemetry.addData("rangeLeft", robot.rangeLeft);
            telemetry.addData("rangeRight", robot.rangeRight);
            telemetry.update();

            if (gamepad1.x) {
                robot.move(1.0, 2.0);
            }

            while(opModeIsActive()){

                robot.goalShootAlign(0.45);

                telemetry.addData("Status", "Running");
                telemetry.addData("rangeFront", robot.rangeFront);
                telemetry.addData("rangeLeft", robot.rangeLeft);
                telemetry.addData("rangeRight", robot.rangeRight);
                telemetry.update();
            }
        }
    }
}
