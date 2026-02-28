package frc.util;

import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.ShooterConstants;

public class ShooterUtil {
    
    // https://www.desmos.com/calculator/4mup6yydoo
    public static double calculateLinearLaunchVelocity(double distance, double initialHeight, double center_offset) {
      double angle = Math.toRadians(75);

      double velocity = ((distance+center_offset)/(Math.cos(angle))) * 
          Math.sqrt(9.81/
              (2*((distance+center_offset)*Math.tan(angle)+(initialHeight-ShooterConstants.hubHeight))));
      
      return velocity;
    }

    public static double getDistanceToHub(Pose2d robotPose) {
      if (getHubTranslation2d() != null) {
        return robotPose.getTranslation().getDistance(getHubTranslation2d());
      }

      return 0;      
    }

    public static Translation2d getHubTranslation2d() {
      Optional<Alliance> alliance = DriverStation.getAlliance();

      if (alliance.isEmpty()) {
        return null;
      }

      return (alliance.get() == Alliance.Blue ? FieldConstants.blueHubCenterPose : 
        FieldConstants.redHubCenterPose);
    }

    public static void updateTrajectory(Pose2d robotPose, double height) {
      double distanceToHub = getDistanceToHub(robotPose);

      double angle = Math.toRadians(75);

      double launchVelocity = calculateLinearLaunchVelocity(distanceToHub, height, 0);

      Translation3d[] trajectory = new Translation3d[10];

      for (int i = 0; i < trajectory.length; i++) {
        double t = 0.25*i;

        double z = height+launchVelocity*Math.sin(angle)*t+(-0.5*9.81)*(t*t);

        //horizontal distance 
        double d_horiz = launchVelocity*Math.cos(angle)*t;

        double x = robotPose.getX()+d_horiz*Math.cos(robotPose.getRotation().getRadians());

        double y = robotPose.getY()+d_horiz*Math.sin(robotPose.getRotation().getRadians());

        trajectory[i] = new Translation3d(x, y, z);
      }

      Logger.recordOutput("Trajectory", trajectory);
    }

}
