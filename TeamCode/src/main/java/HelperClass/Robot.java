package HelperClass;


import android.os.SystemClock;
import android.util.Log;
import java.util.concurrent.TimeUnit;

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



public class Robot {

    /* Public OpMode members. */

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
    //b4 90
    public float countPerInch = 90;
    public float inchesPer360 = 79.0116f;
    public float movePower = 0;
    public int moveStep = 0;



    public enum movementDirection {
        noMovement, moveForward, moveBackward, strafeLeft, strafeRight, turnLeft, turnRight, pivotRotation }


    public static BNO055IMU imu;
    public Orientation angles;
    public Acceleration gravity;
    public static double startingAngle=0;



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
        //FLMotor = hwMap.get(DcMotor.class, "FLMotor");
        //BLMotor = hwMap.get(DcMotor.class, "BLMotor");
        //FRMotor = hwMap.get(DcMotor.class, "FRMotor");
        //BRMotor = hwMap.get(DcMotor.class, "BRMotor");
        FLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //FLMotor.setDirection(DcMotor.Direction.FORWARD);
        //BLMotor.setDirection(DcMotor.Direction.FORWARD);
        //FRMotor.setDirection(DcMotor.Direction.REVERSE);
        //BRMotor.setDirection(DcMotor.Direction.REVERSE);
        FLMotor.setDirection(DcMotor.Direction.REVERSE);
        BLMotor.setDirection(DcMotor.Direction.REVERSE);
        FRMotor.setDirection(DcMotor.Direction.FORWARD);
        BRMotor.setDirection(DcMotor.Direction.FORWARD);
        intake.setDirection(DcMotor.Direction.FORWARD);
        shooting1.setDirection(DcMotor.Direction.REVERSE);
        shooting2.setDirection(DcMotor.Direction.FORWARD);
        arm.setDirection(DcMotor.Direction.FORWARD);
        //servos
        ringPush.setDirection(Servo.Direction.FORWARD);
        intakeHolder.setDirection(Servo.Direction.FORWARD);
        claw.setDirection(Servo.Direction.FORWARD);

        FLMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BLMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FRMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BRMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        FLMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        BLMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        FRMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        BRMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        shooting1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        shooting2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        arm.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

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

    public void setupStartPosition () throws InterruptedException {

        // TODO: Lift wobbleGoal , Shoot, Intake motors to be setup here

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

    /*
    // Direction coule front , back , left strafing , right strafing
    public void moveRobot (float d,float v, float angle1) {

            initRobotAngle=angle1;
            if ((getRobotAngle() > 315) || (getRobotAngle() < 45)) moveTarget = d-fieldY;
            else if ((getRobotAngle() > 135) && (getRobotAngle() < 225)) moveTarget = fieldY -d;
            else if ((getRobotAngle() > 45) && (getRobotAngle() < 135)) moveTarget = d - fieldX;
            else moveTarget = fieldX - d;
            movePower.setAll(v);
            moveStep=MoveStep.moveHoldAngle;

    }

     */
    //

    public void moveRobot () {

        moveStep = 1;

    }


   /* public void stopRobot ()  {
        FLMotor.setPower(0);
        BLMotor.setPower(0);
        FRMotor.setPower(0);
        BRMotor.setPower(0);
    }
*/

    public void strafe()  {
        moveStep = 6;
    }

    public void shoot(){
        moveStep = 31;
    }
    public void intake(){
        moveStep = 30;
    }




    public void moveUpdate() {

        switch (moveStep) {
            case 0:
                break; //End everything

            //MOVE THE ROBOT
            case 1:
                FLMotor.setPower(movePower);
                BLMotor.setPower(movePower);
                FRMotor.setPower(movePower);
                BRMotor.setPower(movePower);
                moveStep = 11; //this and the break statement can be removed to no effect
                break;
            case 11:

                if (Math.abs(moveTarget*countPerInch * .1) - Math.abs(FLMotor.getCurrentPosition()) <= 0) {
                    FLMotor.setPower(movePower);
                    BLMotor.setPower(movePower);
                    FRMotor.setPower(movePower);
                    BRMotor.setPower(movePower);
                    moveStep = 12; //this and the break statement can be removed to no effect

                }
                break;
            case 12:

                if (Math.abs(moveTarget*countPerInch) - Math.abs(FLMotor.getCurrentPosition()) <= 0) {
                    FLMotor.setPower(movePower);
                    BLMotor.setPower(movePower);
                    FRMotor.setPower(movePower);
                    BRMotor.setPower(movePower);
                    moveStep = 2; //this and the break statement can be removed to no effect

                }
                break;
            case 2:

            case 9:

                if (Math.abs(moveTarget*countPerInch) - Math.abs(FLMotor.getCurrentPosition()) <= 0) {
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
                    moveStep = 3;
                }
                break;
            //END OF MOVING THE ROBOT

            case 3:
                moveStep = 0;
                break;

            //STRAFE
            case 6:
                FLMotor.setPower(movePower);
                BLMotor.setPower(-movePower);
                FRMotor.setPower(-movePower);
                BRMotor.setPower(movePower);
                moveStep = 9;
                break;

            //TURNS THE ROBOT
            case 10:
                if(moveTarget-getRobotAngle() < 0){
                    FLMotor.setPower(movePower);
                    BLMotor.setPower(movePower);
                    FRMotor.setPower(-movePower);
                    BRMotor.setPower(-movePower);
                    moveStep = 20;
                }
                else{
                    FLMotor.setPower(-movePower);
                    BLMotor.setPower(-movePower);
                    FRMotor.setPower(movePower);
                    BRMotor.setPower(movePower);
                    moveStep = 21;
                }
                break;

            case 20:
                if (moveTarget - getRobotAngle()  >= -2) {
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
                    moveStep = 3;
                }
                break;

            case 21:
                if (moveTarget - getRobotAngle() <= 2) {
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
                    moveStep = 3;
                }
                break;
            case 30:
                intake.setPower(movePower);
                break;
            case 31:
                shooting1.setPower(movePower);
                shooting2.setPower(movePower);
                break;


            default:
                moveStep = 0;
                break;
        }

    }


}





