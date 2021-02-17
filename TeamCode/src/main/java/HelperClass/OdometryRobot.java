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

import java.util.concurrent.TimeUnit;



public class OdometryRobot {

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

    public static final double oneRotationTicks = 8192;
    public static final double countsPerInch = 1304.45;

    public static final double wheelRadius = 1.0; // In INCH
    public static final double wheelDistanceApart = 15.2; // distance between left and right - in INCH
    public static final double horizontalTicksPerRadian = -134;

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

        // Since our Rev Expansion is in Vertical Position, so we need to Z & X

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


    public void RobotOdometer() {

    }


    public void resetTicks() {
        resetLeftTicks();
        resetCenterTicks();
        resetRightTicks();
    }
    public void resetLeftTicks() {
        leftEncoderPos = leftEncoderMotor.getCurrentPosition();
    }
    public int getLeftTicks() {
        return leftEncoderMotor.getCurrentPosition() - leftEncoderPos;
    }
    public void resetRightTicks() {
        rightEncoderPos = rightEncoderMotor.getCurrentPosition();
    }
    public int getRightTicks() {
        return rightEncoderMotor.getCurrentPosition() - rightEncoderPos;
    }
    public void resetCenterTicks() {
        centerEncoderPos = centerEncoderMotor.getCurrentPosition();
    }
    public int getCenterTicks() {
        return centerEncoderMotor.getCurrentPosition() - centerEncoderPos;
    }
    public void drive(double fl, double bl, double fr, double br) {
        FLMotor.setPower(fl);
        BLMotor.setPower(bl);
        FRMotor.setPower(fr);
        BRMotor.setPower(br);
    }
    public void updatePosition() {
        deltaLeftDistance = (getLeftTicks() / oneRotationTicks) * 2.0 * Math.PI * wheelRadius;
        deltaRightDistance = (getRightTicks() / oneRotationTicks) * 2.0 * Math.PI * wheelRadius;
        deltaCenterDistance = (getCenterTicks() / oneRotationTicks) * 2.0 * Math.PI * wheelRadius;
        x  += (((deltaLeftDistance + deltaRightDistance) / 2.0)) * Math.cos(theta);
        y  += (((deltaLeftDistance + deltaRightDistance) / 2.0)) * Math.sin(theta);
        theta  += (deltaLeftDistance - deltaRightDistance) / wheelDistanceApart;
        //resetTicks();
    }
    public double getX() {
        return x;
    }
    public double getY() {
        return y;
    }
    public double getTheta() {
        return theta;
    }
    public void setX(double _x) {
        x = _x;
    }
    public void setY(double _y) {
        y = _y;
    }
    public void setTheta(double _theta) {
        theta = _theta;
    }
    public double angle() {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle);
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

}










