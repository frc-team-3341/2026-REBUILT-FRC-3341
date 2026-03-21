package frc.util;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.FieldConstants;

public class ClimberUtil {
    
    /**
     * Gets the left and right tower poses depending on the selected alliance
     * 
     * @return
     * returns the left and right tower Pose2ds depending on if the selected alliance is red or
     * blue. If no alliance is selected, this method returns null.
     * 
     */
    public static Pose2d[] getTowerPoses() {
        Optional<Alliance> alliance = DriverStation.getAlliance();

        if (alliance.isEmpty()) {
            return null;
        }

        Pose2d[] towerPoses = new Pose2d[2];

        if (alliance.get() == Alliance.Red) {
            towerPoses[0] = FieldConstants.redLeftTowerPose;
            towerPoses[1] = FieldConstants.redRightTowerPose;

            return towerPoses;
        }

        towerPoses[0] = FieldConstants.blueLeftTowerPose;
        towerPoses[1] = FieldConstants.blueRightTowerPose;

        return towerPoses;
    }
}