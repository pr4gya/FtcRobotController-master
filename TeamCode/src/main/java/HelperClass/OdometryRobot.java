package HelperClass;

import android.os.SystemClock;
import android.util.Log;
import java.util.concurrent.TimeUnit;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.MotorControlAlgorithm;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.concurrent.TimeUnit;

public class OdometryRobot {

    public ModernRoboticsI2cRangeSensor rangeFront, rangeLeft, rangeRight;

    /* Rev Expansion IMU Sensors */
    public static BNO055IMU imu;
    BNO055IMU.Parameters parameters;
    public Orientation angles;
    public Acceleration gravity;



    /* local OpMode members. */
    HardwareMap hwMap = null;

    private ElapsedTime runtime = new ElapsedTime();
    private double currentTime = runtime.seconds();

    //static final DcMotor.ZeroPowerBehavior ZERO_POWER_BEHAVIOR = DcMotor.ZeroPowerBehavior.BRAKE;


    /* Odometer variables  */
    /**
     * plug left encoder into frontleft, right encoder into frontright, center encoder into backleft (arbitary assignments)
     */
    // The amount of encoder ticks for each inch the robot moves. THIS WILL CHANGE FOR EACH ROBOT AND NEEDS TO BE UPDATED HERE
    // Using Rev Bore Encoder with 8192 Counts per revolution https://www.revrobotics.com/rev-11-1271/
    // Deadwheel Diamater of 2 Inch
    // Hence Count_per_inch = 8192 / ( 2* Pi * radius )

    public static final double oneRotationTicks = Constants.ODOMETRY_TICKS_PER_REV;
    public static final double countsPerInch = Constants.ODOMETRY_COUNTS_PER_INCH;
    public static final double wheelRadius = Constants.ODOMETRY_WHEEL_RADIUS;
    public static final double wheelDistanceApart = Constants.ODOMETRY_WHEEL_DISTANCE_INCH; // distance between left and right - in INCH
    public static final double horizontalTicksPerRadian = Constants.HORIZONTAL_TICKS_PER_RAD;


    private int leftEncoderPos = 0;
    private int centerEncoderPos = 0;
    private int rightEncoderPos = 0;
    private double deltaLeftDistance = 0;
    private double deltaRightDistance = 0;
    private double deltaCenterDistance = 0;
    private double x = 0;
    private double y = 0;
    private double theta = 0;

    /* Public opMode members */

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
    public final static double CLAW_HOME = 0.0;
    public final static double CLAW_MAX = 1.0;

    boolean tRight;
    public DcMotor leftEncoderMotor = null;
    public DcMotor rightEncoderMotor = null;
    public DcMotor centerEncoderMotor = null;

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



        // Odometer linking to motors
        leftEncoderMotor = hwMap.get(DcMotor.class, "FLMotor");
        rightEncoderMotor = hwMap.get(DcMotor.class, "FRMotor");
        centerEncoderMotor = hwMap.get(DcMotor.class, "BLMotor");

        leftEncoderMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightEncoderMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        centerEncoderMotor.setDirection(DcMotorSimple.Direction.FORWARD);

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

        FLMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BLMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FRMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BRMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        FLMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BLMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FRMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BRMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftEncoderMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightEncoderMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        centerEncoderMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftEncoderMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightEncoderMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        centerEncoderMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


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

        // Range Sensor
        rangeFront = hwMap.get(ModernRoboticsI2cRangeSensor.class, "range_front"); // I2C Port -1
        rangeRight = hwMap.get(ModernRoboticsI2cRangeSensor.class, "range_right"); // I2C Port -2


    }


    public void RobotOdometer() {

    }
//autoInit() initializes the robot
    public void autoInit() {
        FLMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FRMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BLMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BRMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //turnPid.setKp(1.8);
        //turnPid.setKi(0.0);
        //turnPid.setKd(1.8);
    }

    //in move() we pass 2 variables, forward and turn
    public void move(double forward, double turn) {
        FLMotor.setPower(forward - turn);
        FRMotor.setPower(forward + turn);
        BLMotor.setPower(forward - turn);
        BRMotor.setPower(forward + turn);
    }

    public void setPowers(double fl, double fr, double bl, double br) {
       FLMotor.setPower(fl);
       FRMotor.setPower(fr);
       BLMotor.setPower(bl);
       BRMotor.setPower(br);
    }

//
    public void setPowers(double[] powers) {
        setPowers(powers[0], powers[1], powers[2], powers[3]);
    }

    //
    public void setDrivetrainMode(DcMotor.RunMode runMode) {
        FLMotor.setMode(runMode);
        FRMotor.setMode(runMode);
        BLMotor.setMode(runMode);
        BRMotor.setMode(runMode);
    }

    public void robotStop() {
        // set powers to 0
        move(0, 0);
    }

    public void robotRun(double forward, double turn) {
        setPowers(forward - turn, forward + turn, forward - turn, forward + turn);
    }
    public void robotTurn(boolean tRight, double tAngle, double dAngle, double fwdpwr){
        float angleOne = getRobotAngle();
        dAngle = 200;

        if(dAngle>angleOne){
            tRight = true;
        }
        else{
            tRight = false;
        }
        while (!tRight && dAngle != angleOne ){
            angleOne = getRobotAngle();
            FLMotor.setPower(-fwdpwr);
            BLMotor.setPower(fwdpwr);
            FRMotor.setPower(-fwdpwr);
            BRMotor.setPower(fwdpwr);
        }
        while (tRight && dAngle != angleOne ){
            angleOne = getRobotAngle();
            FLMotor.setPower(fwdpwr);
            BLMotor.setPower(-fwdpwr);
            FRMotor.setPower(fwdpwr);
            BRMotor.setPower(-fwdpwr);
        }
    }


    //fieldcentric movement
    public void fieldCentricMove(double x, double y, double turn) {

        x *= -1.0;
        double power = Math.hypot(x, y);
        double theta = Math.atan2(y, x) - imuAngleInRad(); // this should be replaced with odometer angle

        double rx = (Math.sin(theta + (Math.PI / 4))) * power;
        double lx = (Math.sin(theta - (Math.PI / 4))) * power;

        double fl = lx - turn;
        double fr = rx + turn;
        double bl = rx - turn;
        double br = lx + turn;

        setPowers(fl, fr, bl, br);

    }

    //imu angle in degrees
    public double imuAngleInDeg() {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle);
    }

    public float imuRobotAngle() {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        if (angles.firstAngle < 0) angles.firstAngle = angles.firstAngle * (-1.0f);
        else angles.firstAngle = -angles.firstAngle + 360.0f;
        return (angles.firstAngle);
    }

    public float getRobotAngle() {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        if (angles.firstAngle < 0) angles.firstAngle = angles.firstAngle * (-1.0f);
        else angles.firstAngle = -angles.firstAngle + 360.0f;
        return (angles.firstAngle);
    }


    //imu angle in radians
    public float imuAngleInRad() {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
        return (angles.firstAngle);
    }

    /*
     * This method uses range sensor to adjust the distance from sidewall & front wall and always in right place to shoot high goal
     * Step 1 - Checks the frontwall senssor and adjust the front or back movements
     * Step 2 - Checks the sidewall sensor and adjust the left or right movement
     * Step 3 - Adjust the angle to make sure always IMU angle is between 359 - 360 or 0 - 1 degree
     *
     *
     */
    public void goalShootAlign(double speed) {
        double distanceFront = 1;
        double distanceLeft = 1;
        double distanceRight = 1; // currently we are not using right sensor
        distanceFront = rangeFront.getDistance(DistanceUnit.CM);
        distanceRight = rangeRight.getDistance(DistanceUnit.CM);
        if (distanceFront > Constants.FRONT_RANGE_CM) {
            // go forward
            FLMotor.setPower(speed);
            BLMotor.setPower(speed);
            FRMotor.setPower(speed);
            BRMotor.setPower(speed);
            while (distanceFront > Constants.FRONT_RANGE_CM) {
                // Check front distance from wall
                distanceFront = rangeFront.getDistance(DistanceUnit.CM);
            }
            if (distanceFront <= Constants.FRONT_RANGE_CM) {
                robotStop();
            }
        } else if (distanceFront < Constants.FRONT_RANGE_CM) {
            // move back
            FLMotor.setPower(-speed);
            BLMotor.setPower(-speed);
            FRMotor.setPower(-speed);
            BRMotor.setPower(-speed);
            while (distanceFront < Constants.FRONT_RANGE_CM) {
                // Check front distance from wall
                distanceFront = rangeFront.getDistance(DistanceUnit.CM);
            }
            if (distanceFront >= Constants.FRONT_RANGE_CM) {
                robotStop();
            }
        }
        distanceRight = rangeRight.getDistance(DistanceUnit.CM);
        SystemClock.sleep(250);
        // Step -2 Now adjust the side distance
        if (distanceRight > Constants.RIGHT_RANGE_CM) {
            // move right
            FLMotor.setPower(speed);
            BLMotor.setPower(-speed);
            FRMotor.setPower(-speed);
            BRMotor.setPower(speed);
            while (distanceRight > Constants.RIGHT_RANGE_CM) {
                // Check right distance from wall
                distanceRight = rangeRight.getDistance(DistanceUnit.CM);
            }
            if (distanceRight <= Constants.RIGHT_RANGE_CM) {
                robotStop();
            }
        } else if (distanceRight < Constants.RIGHT_RANGE_CM) {
            // move left
            FLMotor.setPower(-speed);
            BLMotor.setPower(speed);
            FRMotor.setPower(speed);
            BRMotor.setPower(-speed);
            while (distanceRight < Constants.RIGHT_RANGE_CM) {
                // Check right distance from wall
                distanceRight = rangeRight.getDistance(DistanceUnit.CM);
            }
            if (distanceRight >= Constants.RIGHT_RANGE_CM) {
                robotStop();
            }
        }
    }

}










