package HelperClass;


import android.os.SystemClock;
import android.util.Log;
import java.util.concurrent.TimeUnit;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.MotorControlAlgorithm;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;

//import com.sun.tools.javac.tree.DCTree;
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import org.firstinspires.ftc.teamcode.Vortex14969RedAutonomous;

public class Robot {

    /* public opmode members. */

    public DcMotor FLMotor = null;
    public DcMotor FRMotor = null;
    public DcMotor BLMotor = null;
    public DcMotor BRMotor = null;
    public DcMotor intake = null;
    public DcMotor shooting1 = null;
    public DcMotor shooting2 = null;
    public Servo intakeHolder = null;
    public Servo ringPush = null;
    public DcMotor arm = null;
    public Servo claw = null;

    public float moveTarget = 0;

    public float countPerInch = 90;
    public float inchesPer360 = 79.0116f;
    public float movePower = 1;
    public int moveStep = 0;

    public float maxSpeed = 0.5f;
    public float medSpeed = 0.25f;


    public float strafeSpeed = 0.25f;
    public float sidedistR = 0.0f;
    public float fwdDistR = 0.0f;
    public float speedR = 0.0f;
    public float strafeStartDist = 0.0f;

    public float straightLDR = 0.0f;
    public float straightspeedR = 0.0f;
    public float sideLDR = 0.0f;
    public float sidespeedLR = 0.0f;


    public final static double CLAW_HOME = 0.0;
    public final static double CLAW_MIN = 0.0;
    public final static double CLAW_MAX = 1.0;

    public ElapsedTime runtime1 = new ElapsedTime();
    public double currentTime1 = runtime1.seconds();

    public static BNO055IMU imu;
    public Orientation angles;
    public Acceleration gravity;
    public static double startingAngle=0;
    private float angleLast;
//    public DistanceSensor frontRange;
//    public DistanceSensor redWallRange;

    int autonomousStep;

    public enum AutoStep {
        strafeStart, strafeStop, fwdMv, fwdMvStop, wbleMv, wbleMvStop, shootCont, end,
        dropwbl, wbleStart, wbleStop, launchlinestart , launchlinestop, launchlineend, launchlinestop2,
        alignToShootRangeStart, alignToShootRangeStop, readyToShoot, shootStart, shootEnd, takePowerShoot, teleopAlign, park, teleopAlignStop

    }

    double offset;

    public AutoStep autostep = AutoStep.end;

    /* local OpMode members. */
    HardwareMap hwMap = null;
    public ElapsedTime period = new ElapsedTime();


    public void Robot() {

    }

    public void setrobotradius(double rad) {
        inchesPer360 = (float) (rad * 2 * Math.PI);
    }

    public void init(HardwareMap ahwMap) throws InterruptedException {

        hwMap = ahwMap;
        //controller a
        FLMotor = hwMap.get(DcMotorEx.class, "FLMotor");
        BLMotor = hwMap.get(DcMotorEx.class, "BLMotor");
        FRMotor = hwMap.get(DcMotorEx.class, "FRMotor");
        BRMotor = hwMap.get(DcMotorEx.class, "BRMotor");
        //controller b
        intake = hwMap.get(DcMotorEx.class, "intake");
        shooting1 = hwMap.get(DcMotorEx.class, "shooting1");
        shooting2 = hwMap.get(DcMotorEx.class, "shooting2");
        arm = hwMap.get(DcMotorEx.class,"arm" );

        intakeHolder = hwMap.get(Servo.class, "intakeHolder");
        ringPush = hwMap.get(Servo.class, "ringPush");
        claw = hwMap.get(Servo.class, "claw");


        // init rev 2m dist sensor
        // I2C Port - 2 Front Range Sensor connected
//        frontRange = hwMap.get(DistanceSensor.class, "range_front");
        // I2C Port - 1 Red Wall Range Sensor connected
//        redWallRange = hwMap.get(DistanceSensor.class, "range_red_wall");


        //FLMotor = hwMap.get(DcMotor.class, "FLMotor");
        //BLMotor = hwMap.get(DcMotor.class, "BLMotor");
        //FRMotor = hwMap.get(DcMotor.class, "FRMotor");
        //BRMotor = hwMap.get(DcMotor.class, "BRMotor");

        //FLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //BLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //FRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //BRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //shooting1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //shooting2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //FLMotor.setDirection(DcMotor.Direction.FORWARD);
        //BLMotor.setDirection(DcMotor.Direction.FORWARD);
        //FRMotor.setDirection(DcMotor.Direction.REVERSE);
        //BRMotor.setDirection(DcMotor.Direction.REVERSE);
        FLMotor.setDirection(DcMotor.Direction.REVERSE);
        BLMotor.setDirection(DcMotor.Direction.REVERSE);
        FRMotor.setDirection(DcMotor.Direction.FORWARD);
        BRMotor.setDirection(DcMotor.Direction.FORWARD);
        intake.setDirection(DcMotor.Direction.FORWARD);

        shooting1.setDirection(DcMotor.Direction.FORWARD);
        shooting2.setDirection(DcMotor.Direction.REVERSE);

        arm.setDirection(DcMotor.Direction.REVERSE);

        //servos
        ringPush.setDirection(Servo.Direction.FORWARD);
        intakeHolder.setDirection(Servo.Direction.FORWARD);
        claw.setDirection(Servo.Direction.FORWARD);
        claw.setPosition(CLAW_HOME);

//        FLMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        BLMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        FRMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        BRMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
        shooting1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooting2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

//        FLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        BLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        FRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        BRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooting1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooting2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        FLMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        BLMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        FRMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        BRMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        shooting1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        shooting2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        arm.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);



        // I2C Port - 0 IMU Connected
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit            = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit            = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled       = true;
        parameters.useExternalCrystal   = true;
        parameters.mode                 = BNO055IMU.SensorMode.IMU;
        parameters.loggingTag           = "IMU";
        imu                             = hwMap.get(BNO055IMU.class, "imu");

        // need z & x because rev expansion hub is mounted vertically

        //must be in config mode to write to registers
        imu.write8(BNO055IMU.Register.OPR_MODE,BNO055IMU.SensorMode.CONFIG.bVal & 0x0F);
        byte AXIS_MAP_CONFIG_BYTE = 0x6; //This is what to write to the AXIS_MAP_CONFIG register to swap x and z axes
        byte AXIS_MAP_SIGN_BYTE = 0x1; //This is what to write to the AXIS_MAP_SIGN register to negate the z axis
        //Need to be in CONFIG mode to write to registers
        imu.write8(BNO055IMU.Register.OPR_MODE,BNO055IMU.SensorMode.CONFIG.bVal & 0x0F);
        TimeUnit.MILLISECONDS.sleep(100); //changing modes require a delay before doing anything else

        //write to the AXIS_MAP_CONFIG register
        imu.write8(BNO055IMU.Register.AXIS_MAP_CONFIG,AXIS_MAP_CONFIG_BYTE & 0x0F);

        //write to the AXIS_MAP_SIGN register
        imu.write8(BNO055IMU.Register.AXIS_MAP_SIGN,AXIS_MAP_SIGN_BYTE & 0x0F);

        //change back into the IMU mode to use the gyro
        imu.write8(BNO055IMU.Register.OPR_MODE,BNO055IMU.SensorMode.IMU.bVal & 0x0F);

        TimeUnit.MILLISECONDS.sleep(100); //again, requires a delay

        imu.initialize(parameters);


    }

    public void setupStartPosition () throws InterruptedException {

        // TODO: lift, wobble goal, shoot, intake motor setup

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit            = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit            = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled       = true;
        parameters.useExternalCrystal   = true;
        parameters.mode                 = BNO055IMU.SensorMode.IMU;
        parameters.loggingTag           = "IMU";
        imu                             = hwMap.get(BNO055IMU.class, "imu");

        //Need to be in CONFIG mode to write to registers
        imu.write8(BNO055IMU.Register.OPR_MODE,BNO055IMU.SensorMode.CONFIG.bVal & 0x0F);
        byte AXIS_MAP_CONFIG_BYTE = 0x6; //This is what to write to the AXIS_MAP_CONFIG register to swap x and z axes
        byte AXIS_MAP_SIGN_BYTE = 0x1; //This is what to write to the AXIS_MAP_SIGN register to negate the z axis
        //Need to be in CONFIG mode to write to registers
        imu.write8(BNO055IMU.Register.OPR_MODE,BNO055IMU.SensorMode.CONFIG.bVal & 0x0F);
        TimeUnit.MILLISECONDS.sleep(100); //Changing modes requires a delay before doing anything else

        //Write to the AXIS_MAP_CONFIG register
        imu.write8(BNO055IMU.Register.AXIS_MAP_CONFIG,AXIS_MAP_CONFIG_BYTE & 0x0F);

        //Write to the AXIS_MAP_SIGN register
        imu.write8(BNO055IMU.Register.AXIS_MAP_SIGN,AXIS_MAP_SIGN_BYTE & 0x0F);

        //Need to change back into the IMU mode to use the gyro
        imu.write8(BNO055IMU.Register.OPR_MODE,BNO055IMU.SensorMode.IMU.bVal & 0x0F);

        TimeUnit.MILLISECONDS.sleep(100); //Changing modes again requires a delay

        imu.initialize(parameters);
    }

    public float getRobotAngle() {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        if (angles.firstAngle < 0) angles.firstAngle = angles.firstAngle * (-1.0f);
        else angles.firstAngle = -angles.firstAngle + 360.0f;
        return (angles.firstAngle);
    }

    public float getRobotAngleRad() {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
        return (angles.firstAngle);
    }



    public void moveRobot () {
//        FLMotor.setPower(0.25);
//        BLMotor.setPower(0.25);
//        FRMotor.setPower(0.25);
//        BRMotor.setPower(0.25);
        moveStep = 1;
    }



    public void strafe()  {
        moveStep = 6;
    }
    public void strafe2(){
        moveStep = 40;
    }
    public void mvFwd(){
        moveStep = 41;
    }
    public void shoot(){
        moveStep = 31;
    }
    public void intake(){
        moveStep = 30;
    }

    public void mvDestination(float fwdDist, float speed, float sideDist){
        strafeStartDist = 6.0f;
        fwdDistR = fwdDist;
        strafeSpeed = -0.25f;
        speedR = speed;
        sidedistR = sideDist;
        autostep = AutoStep.strafeStart;

    }

    public void mvlaunchLine(float straightLD, float straightspeed, float sideLD, float sidespeedL) {
        //straight launch distance robot
        straightLDR = straightLD;
        straightspeedR = straightspeed;
        sideLDR = sideLD;
        sidespeedLR = sidespeedL;
      //  autostep = AutoStep.launchlinestart;
    }


    public void adjustAngle () {

    }
    public void moveUpdate() throws InterruptedException {


        switch (autostep){

//            case teleopAlign:
//                angleLast = getRobotAngle();
//                if (360 - angleLast < 90) {
//                    FLMotor.setPower(0.2);
//                    BLMotor.setPower(0.2);
//                    FRMotor.setPower(-0.2);
//                    BRMotor.setPower(-0.2);
//                    autostep = AutoStep.teleopAlignStop;
//
//                } else if (angleLast - 0 < 90) {
//                    FLMotor.setPower(-0.2);
//                    BLMotor.setPower(-0.2);
//                    FRMotor.setPower(0.2);
//                    BRMotor.setPower(0.2);
//                    autostep = AutoStep.teleopAlignStop;
//                }
//                break;
//
//            case teleopAlignStop:
//                if (360 - angleLast  <= 1) {
//                    FLMotor.setPower(0);
//                    BLMotor.setPower(0);
//                    FRMotor.setPower(0);
//                    BRMotor.setPower(0);
//                    FLMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                    BLMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                    FRMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                    BRMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//                    FLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//                    BLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//                    FRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//                    BRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//                    autostep = AutoStep.readyToShoot;
//
//                } else if (angleLast - 0 <= 1) {
//                    FLMotor.setPower(0);
//                    BLMotor.setPower(0);
//                    FRMotor.setPower(0);
//                    BRMotor.setPower(0);
//                    FLMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                    BLMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                    FRMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                    BRMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//                    FLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//                    BLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//                    FRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//                    BRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//                    autostep = AutoStep.end;
//
//                } else {
//                    autostep = AutoStep.teleopAlign;
//                }
//                break;


            case strafeStart:
                FLMotor.setPower(strafeSpeed);
                BLMotor.setPower(-strafeSpeed);
                FRMotor.setPower(-strafeSpeed);
                BRMotor.setPower(strafeSpeed);
                autostep = AutoStep.strafeStop;
                break;

            case strafeStop:
                if (Math.abs(strafeStartDist*countPerInch) - Math.abs(FLMotor.getCurrentPosition()) <= 0) {
                    FLMotor.setPower(0);
                    BLMotor.setPower(0);
                    FRMotor.setPower(0);
                    BRMotor.setPower(0);
                    FLMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    BLMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    FRMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    BRMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                    FLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    BLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    FRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    BRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    strafeStartDist = 0;
                    autostep = AutoStep.fwdMv;
                }
                else{
                    autostep = AutoStep.strafeStart;
                }

                break;


            case fwdMv:
                FLMotor.setPower(speedR);
                BLMotor.setPower(speedR);
                FRMotor.setPower(speedR);
                BRMotor.setPower(speedR);
                autostep = AutoStep.fwdMvStop;
                break;

            case fwdMvStop:
                if (Math.abs(fwdDistR*countPerInch) - Math.abs(FLMotor.getCurrentPosition()) <= 0) {
                    FLMotor.setPower(0);
                    BLMotor.setPower(0);
                    FRMotor.setPower(0);
                    BRMotor.setPower(0);
                    FLMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    BLMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    FRMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    BRMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                    FLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    BLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    FRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    BRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    autostep = AutoStep.wbleMv;
                }
                else{
                    autostep = AutoStep.fwdMv;
                }
                break;


            case wbleMv:
                strafeSpeed = 0.25f;
                FLMotor.setPower(strafeSpeed);
                BLMotor.setPower(-strafeSpeed);
                FRMotor.setPower(-strafeSpeed);
                BRMotor.setPower(strafeSpeed);
                autostep = AutoStep.wbleMvStop;
                break;


            case wbleMvStop:
                if (Math.abs(sidedistR*countPerInch) - Math.abs(FLMotor.getCurrentPosition()) <= 0) {
                    FLMotor.setPower(0);
                    BLMotor.setPower(0);
                    FRMotor.setPower(0);
                    BRMotor.setPower(0);
                    FLMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    BLMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    FRMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    BRMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                    FLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    BLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    FRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    BRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    autostep = AutoStep.wbleStart;
                    break;

                }
                else{
                    autostep = AutoStep.wbleMv;
                    break;
                }

            case wbleStart:
                runtime1.reset();
                currentTime1 = runtime1.seconds();
                arm.setPower(-0.25);
                SystemClock.sleep(1000);
                arm.setPower(0);
                autostep=AutoStep.dropwbl;
                break;

            case dropwbl:
                SystemClock.sleep(1000);
                claw.setPosition(1);
                SystemClock.sleep(1000);
                //if (claw.getPosition()==1) {
                autostep = AutoStep.wbleStop;
                //}

                break;

            case wbleStop:
                arm.setPower(0.3);
                SystemClock.sleep(500);
                arm.setPower(0);
                claw.setPosition(0.5);
                autostep = AutoStep.launchlinestart;
                break;

            case shootCont:
                break;
            case end:
                break;

            case launchlinestart:
                FLMotor.setPower(sidespeedLR);
                BLMotor.setPower(-sidespeedLR);
                FRMotor.setPower(-sidespeedLR);
                BRMotor.setPower(sidespeedLR);
                autostep = AutoStep.launchlinestop;
                break;

            case launchlinestop:
                if (Math.abs(sideLDR*countPerInch) - Math.abs(FLMotor.getCurrentPosition()) <= 0) {
                    FLMotor.setPower(0);
                    BLMotor.setPower(0);
                    FRMotor.setPower(0);
                    BRMotor.setPower(0);

                    FLMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    BLMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    FRMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    BRMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                    FLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    BLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    FRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    BRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    strafeStartDist = 0;
                    if(straightLDR==0){
                        autostep= AutoStep.alignToShootRangeStart;
                    }
                    else{
                        autostep = AutoStep.launchlinestop2;
                    }
                }
                else{
                    autostep = AutoStep.launchlinestart;
                }
                break;


            case launchlineend:
                FLMotor.setPower(straightspeedR);
                BLMotor.setPower(straightspeedR);
                FRMotor.setPower(straightspeedR);
                BRMotor.setPower(straightspeedR);
                autostep = AutoStep.launchlinestop2;
                break;

            case launchlinestop2:
                if (Math.abs(straightLDR*countPerInch) - Math.abs(FLMotor.getCurrentPosition()) <= 0) {
                    FLMotor.setPower(0);
                    BLMotor.setPower(0);
                    FRMotor.setPower(0);
                    BRMotor.setPower(0);
                    FLMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    BLMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    FRMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    BRMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                    FLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    BLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    FRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    BRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    autostep = AutoStep.alignToShootRangeStart;
                }
                else{
                    autostep = AutoStep.launchlineend;
                }
                break;


            case alignToShootRangeStart:

                angleLast = getRobotAngle();

                if (360 - angleLast < 90) {
                    FLMotor.setPower(0.2);
                    BLMotor.setPower(0.2);
                    FRMotor.setPower(-0.2);
                    BRMotor.setPower(-0.2);
                    autostep = AutoStep.alignToShootRangeStop;

                } else if (angleLast - 0 < 90) {
                    FLMotor.setPower(-0.2);
                    BLMotor.setPower(-0.2);
                    FRMotor.setPower(0.2);
                    BRMotor.setPower(0.2);
                    autostep = AutoStep.alignToShootRangeStop;
                }


                break;


            case alignToShootRangeStop:

                angleLast = getRobotAngle();

                if (360 - angleLast  <= 1) {
                    FLMotor.setPower(0);
                    BLMotor.setPower(0);
                    FRMotor.setPower(0);
                    BRMotor.setPower(0);
                    FLMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    BLMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    FRMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    BRMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                    FLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    BLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    FRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    BRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    autostep = AutoStep.readyToShoot;

                } else if (angleLast - 0 <= 1) {
                    FLMotor.setPower(0);
                    BLMotor.setPower(0);
                    FRMotor.setPower(0);
                    BRMotor.setPower(0);
                    FLMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    BLMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    FRMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    BRMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                    FLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    BLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    FRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    BRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    autostep = AutoStep.readyToShoot;

                } else {
                    autostep = AutoStep.alignToShootRangeStop;
                }
                break;

            case readyToShoot:
                SystemClock.sleep(1000);
                autostep = AutoStep.shootStart;
                break;

            case shootStart:
//                ringPush.setPosition(0);
                shooting1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                shooting2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                shooting1.setPower(-0.745);
                shooting2.setPower(-0.745);
                SystemClock.sleep(2500);
                ringPush.setPosition(0.8);
                SystemClock.sleep(1000);
                ringPush.setPosition(0);
                SystemClock.sleep(1000);
                ringPush.setPosition(0.8);
                SystemClock.sleep(1000);
                ringPush.setPosition(0);
//                intake.setPower(-2);
                SystemClock.sleep(1000);
                ringPush.setPosition(0.8);
                SystemClock.sleep(1000);
                autostep=AutoStep.shootEnd;
                break;

            case takePowerShoot:
                break;

            case shootEnd:
//               SystemClock.sleep(500);
                shooting1.setPower(0);
                shooting2.setPower(0);
                autostep=AutoStep.park;
                break;

            case teleopAlign:
                break;
            case park:
                FLMotor.setPower(0.5);
                BLMotor.setPower(0.5);
                FRMotor.setPower(0.5);
                BRMotor.setPower(0.5);
                SystemClock.sleep(750);
                FLMotor.setPower(0);
                BLMotor.setPower(0);
                FRMotor.setPower(0);
                BRMotor.setPower(0);
                autostep=AutoStep.end;
                break;






        }
    }

}










