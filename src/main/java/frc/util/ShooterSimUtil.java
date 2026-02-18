package frc.util;

import frc.robot.Constants.ShooterConstants;

public class ShooterSimUtil {
    

    public static double calculateLinearLaunchVelocity(double distance, double initialHeight, double center_offset) {
      double angle = Math.toRadians(75);

      double velocity = ((distance+center_offset)/(Math.cos(angle))) * 
          Math.sqrt(9.81/
              (2*((distance+center_offset)*Math.tan(angle)+(initialHeight-ShooterConstants.hubHeight))));
      
      return velocity;
    }

    // public static double getDistanceToHub() {

    // }

}
