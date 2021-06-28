package HelperClass;

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


        while(!opModeIsActive() && !isStopRequested()){
            telemetry.addData("Status", "Initialized 1");
            telemetry.addData("rangeFront", robot.rangeFront);
            telemetry.addData("rangeLeft", robot.rangeLeft);
            telemetry.addData("rangeRight", robot.rangeRight);
            telemetry.update();



        }

        while(opModeIsActive()){

            telemetry.addData("Status", "Running");
            telemetry.update();

        }
        OdometryRobot robot = new OdometryRobot();

        if (gamepad1.x) {
            robot.move(1.0, 2.0);
        }
    }
}
