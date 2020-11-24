package Hardware;

import android.os.SystemClock;
import com.qualcomm.robotcore.hardware.DcMotor;


public class DriveTrainNew {


    // Put this code in a new DriveTrain Class
    // The following code is from GoBilda Sample code
    // https://gm0.org/en/latest/docs/software/mecanum-drive.html
    public static double move_y_axis;
    public static double move_x_axis;
    public static double pivot_turn;

    //these are DriveTrain motors
    public DcMotor frontLeft;
    public DcMotor frontRight;
    public DcMotor backLeft;
    public DcMotor backRight;


    //sets up hardware
    public DriveTrainNew (DcMotor fl, DcMotor fr, DcMotor bl, DcMotor br){
        frontLeft = fl;
        frontRight = fr;
        backLeft = bl;
        backRight = br;


        /** Sets up motors without encoders, for speed */////////
        /*
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        */

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }


    private long lastUpdateTime = 0;


    /**
     * Gets the motion vector from the joystick values.
     * @param leftStickX The left joystick X for sideways movement
     * @param leftStickY The left joystick Y for forward and back
     * @param rightStickX The right joystick X for pivot turning
     * @param rightStickY The right joystick Y - not used
     * @return The Mecanum motion vector.
     */


    public void ApplyMovement (double leftStickX,
                               double leftStickY,
                               double rightStickX,
                               double rightStickY) {

        long currTime = SystemClock.uptimeMillis();
        if(currTime - lastUpdateTime < 16){
            return;
        }
        lastUpdateTime = currTime;

        // Assign the joystick movements
        move_y_axis = -leftStickY ;      // Remember, this is reversed!
        move_x_axis = leftStickX * 1.5; // Counteract imperfect strafing
        pivot_turn  = rightStickX;

        double fl_power = move_y_axis + move_x_axis + pivot_turn;
        double bl_power = move_y_axis - move_x_axis + pivot_turn;
        double fr_power = move_y_axis - move_x_axis - pivot_turn;
        double br_power = move_y_axis + move_x_axis - pivot_turn;

        /**
          Put powers in the range of -1 to 1 only if they aren't already (not
          checking would cause us to always drive at full speed)
        */

        if (Math.abs(fl_power) > 1 || Math.abs(bl_power) > 1 ||
                Math.abs(fr_power) > 1 || Math.abs(br_power) > 1 ) {
            // Find the largest power
            double max = 0;
            max = Math.max(Math.abs(fl_power), Math.abs(bl_power));
            max = Math.max(Math.abs(fr_power), max);
            max = Math.max(Math.abs(br_power), max);

            // Divide everything by max (it's positive so we don't need to worry
            // about signs)
            fl_power /= max;
            bl_power /= max;
            fr_power /= max;
            br_power /= max;
        }


        //now we can set the powers
        frontLeft.setPower(fl_power);
        backLeft.setPower(bl_power);
        frontRight.setPower(fr_power);
        backRight.setPower(br_power);

    }

}





