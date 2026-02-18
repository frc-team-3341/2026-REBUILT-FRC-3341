package frc.util;

import java.util.Optional;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Drive;

public class ShooterUtil {
    

    public static double calculateLinearLaunchVelocity(double distance, double initialHeight, double center_offset) {
      double angle = Math.toRadians(75);

      double velocity = ((distance+center_offset)/(Math.cos(angle))) * 
          Math.sqrt(9.81/
              (2*((distance+center_offset)*Math.tan(angle)+(initialHeight-ShooterConstants.hubHeight))));
      
      return velocity;
    }

    public static double getDistanceToHub(Drive robot) {
        return robot.getPose().getTranslation().getDistance(getHubPose2d());
    }

    public static Translation2d getHubPose2d() {
      Optional<Alliance> alliance = DriverStation.getAlliance();

      if (alliance.isEmpty()) {
        return null;
      }

      return (alliance.get() == Alliance.Blue ? FieldConstants.blueHubCenterPose : 
        FieldConstants.redHubCenterPose);
    }

}
