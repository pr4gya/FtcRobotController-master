package HelperClass;

import android.os.SystemClock;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

import java.util.concurrent.TimeUnit;

import HelperClass.Robot;

public class OdoRobot
{
public DcMotor FLMotor = null;
public DcMotor FRMotor = null;
public DcMotor BLMotor = null;
public DcMotor BRMotor = null;


public float countPerInch = 90;
public float inchesPer360 = 79.0116f;
public float movePower = 1;
public int moveStep = 0;

public float maxSpeed = 0.5f;
public float medSpeed = 0.25f;


public final static double CLAW_HOME = 0.0;
public final static double CLAW_MIN = 0.0;
public final static double CLAW_MAX = 1.0;

public ElapsedTime runtime1 = new ElapsedTime();
public double currentTime1 = runtime1.seconds();

//public static BNO055IMU imu;
//public Orientation angles;
//public Acceleration gravity;
//public static double startingAngle=0;
//private float angleLast;
//public DistanceSensor frontRange;
//public DistanceSensor redWallRange;


public enum AutoStep {
    strafeStart, strafeStop, fwdMv, fwdMvStop, wbleMv, wbleMvStop, shootCont, end,
    dropwbl, wbleStart, wbleStop, launchlinestart , launchlinestop, launchlineend, launchlinestop2,
    alignToShootRangeStart, alignToShootRangeStop, readyToShoot, shootStart, shootEnd, takePowerShoot

}

    public Robot.AutoStep autostep = Robot.AutoStep.end;

    /* local OpMode members. */
    HardwareMap hwMap = null;
    public ElapsedTime period = new ElapsedTime();


    public void OdoRobot() {

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


        // Initailizing Rev 2M Distance sensor
        // I2C Port - 2 Front Range Sensor connected
//        frontRange = hwMap.get(DistanceSensor.class, "range_front");
        // I2C Port - 1 Red Wall Range Sensor connected
//        redWallRange = hwMap.get(DistanceSensor.class, "range_red_wall");


        //FLMotor = hwMap.get(DcMotor.class, "FLMotor");
        //BLMotor = hwMap.get(DcMotor.class, "BLMotor");
        //FRMotor = hwMap.get(DcMotor.class, "FRMotor");
        //BRMotor = hwMap.get(DcMotor.class, "BRMotor");

        FLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //FLMotor.setDirection(DcMotor.Direction.FORWARD);
        //BLMotor.setDirection(DcMotor.Direction.FORWARD);
        //FRMotor.setDirection(DcMotor.Direction.REVERSE);
        //BRMotor.setDirection(DcMotor.Direction.REVERSE);
        FLMotor.setDirection(DcMotor.Direction.REVERSE);
        BLMotor.setDirection(DcMotor.Direction.REVERSE);
        FRMotor.setDirection(DcMotor.Direction.FORWARD);
        BRMotor.setDirection(DcMotor.Direction.FORWARD);


        FLMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BLMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FRMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BRMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        FLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        FLMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        BLMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        FRMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        BRMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);



        // I2C Port - 0 IMU Connected
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit            = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit            = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled       = true;
        parameters.useExternalCrystal   = true;
        parameters.mode                 = BNO055IMU.SensorMode.IMU;
        parameters.loggingTag           = "IMU";
//        imu                             = hwMap.get(BNO055IMU.class, "imu");
//
//        // Since our Rev Expansion is in Vertical Position, so we need to Z & X
//
//        //Need to be in CONFIG mode to write to registers
//        imu.write8(BNO055IMU.Register.OPR_MODE,BNO055IMU.SensorMode.CONFIG.bVal & 0x0F);
//        byte AXIS_MAP_CONFIG_BYTE = 0x6; //This is what to write to the AXIS_MAP_CONFIG register to swap x and z axes
//        byte AXIS_MAP_SIGN_BYTE = 0x1; //This is what to write to the AXIS_MAP_SIGN register to negate the z axis
//        //Need to be in CONFIG mode to write to registers
//        imu.write8(BNO055IMU.Register.OPR_MODE,BNO055IMU.SensorMode.CONFIG.bVal & 0x0F);
//        TimeUnit.MILLISECONDS.sleep(100); //Changing modes requires a delay before doing anything else
//
//        //Write to the AXIS_MAP_CONFIG register
//        imu.write8(BNO055IMU.Register.AXIS_MAP_CONFIG,AXIS_MAP_CONFIG_BYTE & 0x0F);
//
//        //Write to the AXIS_MAP_SIGN register
//        imu.write8(BNO055IMU.Register.AXIS_MAP_SIGN,AXIS_MAP_SIGN_BYTE & 0x0F);
//
//        //Need to change back into the IMU mode to use the gyro
//        imu.write8(BNO055IMU.Register.OPR_MODE,BNO055IMU.SensorMode.IMU.bVal & 0x0F);
//
//        TimeUnit.MILLISECONDS.sleep(100); //Changing modes again requires a delay
//
//        imu.initialize(parameters);
//
//
//    }

//    public void setupStartPosition () throws InterruptedException {
//
//        // TODO: Lift wobbleGoal , Shoot, Intake motors to be setup here
//
//        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
//        parameters.angleUnit            = BNO055IMU.AngleUnit.DEGREES;
//        parameters.accelUnit            = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
//        parameters.loggingEnabled       = true;
//        parameters.useExternalCrystal   = true;
//        parameters.mode                 = BNO055IMU.SensorMode.IMU;
//        parameters.loggingTag           = "IMU";
//        imu                             = hwMap.get(BNO055IMU.class, "imu");
//
//        //Need to be in CONFIG mode to write to registers
//        imu.write8(BNO055IMU.Register.OPR_MODE,BNO055IMU.SensorMode.CONFIG.bVal & 0x0F);
//        byte AXIS_MAP_CONFIG_BYTE = 0x6; //This is what to write to the AXIS_MAP_CONFIG register to swap x and z axes
//        byte AXIS_MAP_SIGN_BYTE = 0x1; //This is what to write to the AXIS_MAP_SIGN register to negate the z axis
//        //Need to be in CONFIG mode to write to registers
//        imu.write8(BNO055IMU.Register.OPR_MODE,BNO055IMU.SensorMode.CONFIG.bVal & 0x0F);
//        TimeUnit.MILLISECONDS.sleep(100); //Changing modes requires a delay before doing anything else
//
//        //Write to the AXIS_MAP_CONFIG register
//        imu.write8(BNO055IMU.Register.AXIS_MAP_CONFIG,AXIS_MAP_CONFIG_BYTE & 0x0F);
//
//        //Write to the AXIS_MAP_SIGN register
//        imu.write8(BNO055IMU.Register.AXIS_MAP_SIGN,AXIS_MAP_SIGN_BYTE & 0x0F);
//
//        //Need to change back into the IMU mode to use the gyro
//        imu.write8(BNO055IMU.Register.OPR_MODE,BNO055IMU.SensorMode.IMU.bVal & 0x0F);
//
//        TimeUnit.MILLISECONDS.sleep(100); //Changing modes again requires a delay
//
//        imu.initialize(parameters);
//    }



//    public void moveRobot () {
////        FLMotor.setPower(0.25);
////        BLMotor.setPower(0.25);
////        FRMotor.setPower(0.25);
////        BRMotor.setPower(0.25);
//        moveStep = 1;
//    }
//
//
//
//    public void strafe()  {
//        moveStep = 6;
//    }
//    public void strafe2(){
//        moveStep = 40;
//    }
//    public void mvFwd(){
//        moveStep = 41;
//    }
//    public void shoot(){
//        moveStep = 31;
//    }
//    public void intake(){
//        moveStep = 30;
//    }
//
//
//
//    public void adjustAngle () {
//
//    }
//    public void moveUpdate() throws InterruptedException {
//
//
//        switch (autostep){
//
//
//
//            case end:
//                break;
//

//            case alignToShootRangeStart:
//
//                angleLast = getRobotAngle();
//
//                if (360 - angleLast < 90) {
//                    FLMotor.setPower(0.2);
//                    BLMotor.setPower(0.2);
//                    FRMotor.setPower(-0.2);
//                    BRMotor.setPower(-0.2);
//                    autostep = Robot.AutoStep.alignToShootRangeStop;
//
//                } else if (angleLast - 0 < 90) {
//                    FLMotor.setPower(-0.2);
//                    BLMotor.setPower(-0.2);
//                    FRMotor.setPower(0.2);
//                    BRMotor.setPower(0.2);
//                    autostep = Robot.AutoStep.alignToShootRangeStop;
//                }
//
//
//                break;
//
//
//            case alignToShootRangeStop:
//
//                angleLast = getRobotAngle();
//
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
//                    autostep = Robot.AutoStep.readyToShoot;
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
//                    autostep = Robot.AutoStep.readyToShoot;
//
//                } else {
//                    autostep = Robot.AutoStep.alignToShootRangeStop;
//                }
//
//                break;


        }


    }












