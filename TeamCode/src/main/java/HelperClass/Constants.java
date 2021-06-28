package HelperClass;

public class Constants {
    //holds constant values in one place rather than defining throughout the code
    public static final String TEAM = "Vortex-14969";
    public static final double Y_FIELD_SIZE_INCH = 144;
    public static final double X_FIELD_SIZE_INCH = 96;
    public static final double TILE_SIZE_INCH = 24;
    public static final double ODOMETRY_TICKS_PER_REV = 8192;
    public static final double ODOMETRY_WHEELBASE = 15.2; // in inches
    public static final double ODOMETRY_WHEEL_RADIUS = 1.0; // in inches
    public static final double ODOMETRY_COUNTS_PER_INCH = 8192 / (2.0 * Math.PI * ODOMETRY_WHEEL_RADIUS); //cpI = odo ticks per rev/ circumference of odo wheel
    public static final double ODOMETRY_WHEEL_DISTANCE_INCH  = 15.2; // distance between left and right encoders in inches
    public static final double HORIZONTAL_TICKS_PER_RAD = 650;
    public static final double FRONT_RANGE_CM = 165;
    public static final double LEFT_RANGE_CM = 60;
    public static final double RIGHT_RANGE_CM = 60;

}

