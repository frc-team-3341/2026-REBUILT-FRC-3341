package frc.robot.util;

public class JoystickUtil {
    

    public static double[] scaleJoystickInputs(double x, double y, double maxSpeed) {
        double mag = Math.sqrt((x*x)+(y*y));
        
        double angle = Math.atan2(y, x);

        double[] result = new double[2];

        mag *= maxSpeed;

        result[0] = angle;
        result[1] = mag;

        return result;

    }
}
