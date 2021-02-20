package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import HelperClass.Constants;

public class OdometryCoordinatePosition implements Runnable {

    //Odometry wheels
    private DcMotor verticalEncoderLeft, verticalEncoderRight, horizontalEncoder;
    private DcMotor leftEncoderMotor;
    private DcMotor rightEncoderMotor;
    private DcMotor centerEncoderMotor;

    private static final double oneRotationTicks = Constants.ODOMETRY_TICKS_PER_REV;
    private static final double countsPerInch = Constants.ODOMETRY_COUNTS_PER_INCH;
    private static final double wheelRadius = Constants.ODOMETRY_WHEEL_RADIUS; // In INCH
    private static final double wheelDistanceApart = Constants.ODOMETRY_WHEEL_DISTANCE_INCH; // in INCH
    private static final double horizontalTicksPerRadian = Constants.HORIZONTAL_TICKS_PER_RAD;

    //Thead run condition
    private boolean isRunning = true;

    //Position variables used for storage and calculations
    private int leftEncoderPos = 0;
    private int centerEncoderPos = 0;
    private int rightEncoderPos = 0;
    private int lastLeftEncoderPos = 0;
    private int lastCenterEncoderPos = 0;
    private int lastRightEncoderPos = 0;
    private double deltaLeftPos = 0;
    private double deltaRightPos = 0;
    private double deltaCenterPos = 0;
    private double x = 0;
    private double y = 0;
    private double theta = 0; // Robot orintation in Radians
    private double robotAngleInRadians = 0;
    private double changeInRobotAngle = 0;  // Change in CenterEncode position

    //private double verticalRightEncoderWheelPosition = 0, verticalLeftEncoderWheelPosition = 0, normalEncoderWheelPosition = 0,  changeInRobotOrientation = 0;
    //private double robotGlobalXCoordinatePosition = 0, robotGlobalYCoordinatePosition = 0, robotOrientationRadians = 0;
    //private double previousVerticalRightEncoderWheelPosition = 0, previousVerticalLeftEncoderWheelPosition = 0, prevNormalEncoderWheelPosition = 0;

    //Algorithm constants
    private double robotEncoderWheelDistanceInTicks;
    private double horizontalEncoderTickPerDegreeOffset;

    //Sleep time interval (milliseconds) for the position update thread
    private int sleepTime;

    //Files to access the algorithm constants
    //private File wheelBaseSeparationFile = AppUtil.getInstance().getSettingsFile("wheelBaseSeparation.txt");
    //private File horizontalTickOffsetFile = AppUtil.getInstance().getSettingsFile("horizontalTickOffset.txt");

    private int leftEncoderPositionMultiplier = 1;
    private int rightEncoderPositionMultiplier = 1;
    private int centerEncoderPositionMultiplier = 1;

    /**
     * Constructor for GlobalCoordinatePosition Thread
     * @param leftEncoder left odometry encoder, facing the vertical direction
     * @param rightEncoder right odometry encoder, facing the vertical direction
     * @param centerEncoder horizontal odometry encoder, perpendicular to the other two odometry encoder wheels
     */
    public OdometryCoordinatePosition (DcMotor leftEncoder, DcMotor rightEncoder, DcMotor centerEncoder){
        leftEncoderMotor = leftEncoder;
        rightEncoderMotor = rightEncoder;
        centerEncoderMotor = centerEncoder;
        sleepTime = 50;

        robotEncoderWheelDistanceInTicks = wheelDistanceApart * countsPerInch;
        //this.horizontalEncoderTickPerDegreeOffset = Double.parseDouble(ReadWriteFile.readFile(horizontalTickOffsetFile).trim());
        horizontalEncoderTickPerDegreeOffset = horizontalTicksPerRadian;
    }


    /**
     * Updates the global (x, y, theta) coordinate position of the robot using the odometry encoders
     */
    private void coordinatePositionUpdate(){
        //Get Current Positions
        leftEncoderPos = (leftEncoderMotor.getCurrentPosition() * leftEncoderPositionMultiplier);
        rightEncoderPos = (rightEncoderMotor.getCurrentPosition() * rightEncoderPositionMultiplier);
        centerEncoderPos = (centerEncoderMotor.getCurrentPosition() * centerEncoderPositionMultiplier);


        deltaLeftPos = leftEncoderPos - lastLeftEncoderPos;
        deltaRightPos = rightEncoderPos - lastRightEncoderPos;
        deltaCenterPos = centerEncoderPos - lastCenterEncoderPos;

        //Calculate Angle
        changeInRobotAngle = (deltaLeftPos - deltaRightPos) / (robotEncoderWheelDistanceInTicks);

        robotAngleInRadians = (robotAngleInRadians + changeInRobotAngle);

        //Get the components of the motion

        double horizontalChange = deltaCenterPos - (changeInRobotAngle * horizontalTicksPerRadian);

        double p = ((deltaRightPos + deltaLeftPos) / 2);
        double n = horizontalChange;

        //Calculate and update the position values
        x = x + (p*Math.sin(robotAngleInRadians) + n*Math.cos(robotAngleInRadians));
        y = y + (p*Math.cos(robotAngleInRadians) - n*Math.sin(robotAngleInRadians));


        lastLeftEncoderPos = leftEncoderPos;
        lastRightEncoderPos = rightEncoderPos;
        lastCenterEncoderPos = centerEncoderPos;

    }



    /**
     * Returns the robot's global x coordinate
     * @return global x coordinate
     */
    public double returnXCoordinate(){ return x; }

    /**
     * Returns the robot's global y coordinate
     * @return global y coordinate
     */
    public double returnYCoordinate(){ return y; }

    /**
     * Returns the robot's global orientation
     * @return global orientation, in degrees
     */
    public double returnAngleDegree(){ return Math.toDegrees(robotAngleInRadians); }


    /**
     * Returns the robot's global orientation
     * @return global orientation, in degrees
     */
    public double returnAngleRadian(){ return robotAngleInRadians; }
    /**
     * Stops the position update thread
     */
    public void stop(){ isRunning = false; }

    public void reverseLeftEncoder(){
        if(leftEncoderPositionMultiplier == 1){
            leftEncoderPositionMultiplier = -1;
        }else{
            leftEncoderPositionMultiplier = 1;
        }
    }

    public void reverseRightEncoder(){
        if(rightEncoderPositionMultiplier == 1){
            rightEncoderPositionMultiplier = -1;
        }else{
            rightEncoderPositionMultiplier = 1;
        }
    }

    public void reverseCenterEncoder(){
        if(centerEncoderPositionMultiplier == 1){
            centerEncoderPositionMultiplier = -1;
        }else{
            centerEncoderPositionMultiplier = 1;
        }
    }


    @Override
    public void run() {
        while(isRunning) {
            coordinatePositionUpdate();
            try {
                Thread.sleep(sleepTime);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }

    }
}
