package Odometry.other;

public class MathFunctions {
    /**
     * makes sure an angle is within the range -180 to 180 degrees
     * @param angle
     * @return
     */
public static double AngleWrap(double angle){
    while(angle < -Math.PI){
        angle+= 2 * Math.PI;
    }
    while(angle > Math.PI){
        angle -= 2 * Math.PI;
    }
    return angle;
}

}
