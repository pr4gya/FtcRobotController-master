package Odometry.other;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import HelperClass.Robot;

public class OdoRobot {
    public static boolean usingComputer = true;
    /**
     * creates simulation
     */


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
        worldXPosition = 50;
        worldYPosition = 50;
        worldAngle_rad = Math.toRadians(-180);
    }
    private double xSpeed = 0;
    private double ySpeed = 0;
    private double turnSpeed = 0;

    public static double worldXPosition;
    public static double worldYPosition;
    public static double worldAngle_rad;

    public double getXPos(){
        return worldXPosition;
    }

    public double getYPos(){
        return worldYPosition;
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

        FLMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BLMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FRMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BRMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //FLMotor.setDirection(DcMotor.Direction.FORWARD);
        //BLMotor.setDirection(DcMotor.Direction.FORWARD);
        //FRMotor.setDirection(DcMotor.Direction.REVERSE);
        //BRMotor.setDirection(DcMotor.Direction.REVERSE);
        FLMotor.setDirection(DcMotor.Direction.REVERSE);
        BLMotor.setDirection(DcMotor.Direction.REVERSE);
        FRMotor.setDirection(DcMotor.Direction.FORWARD);
        BRMotor.setDirection(DcMotor.Direction.FORWARD);




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



        }


    }












