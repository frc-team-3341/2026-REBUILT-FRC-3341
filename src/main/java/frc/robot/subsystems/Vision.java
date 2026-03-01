package frc.robot.subsystems;
import static frc.robot.Constants.VisionConstants.*;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;

import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.List;

public class Vision {
    private final PhotonCamera frontCamera;
    private final PhotonPoseEstimator frontCamPhotonEstimator;
    private Matrix<N3, N1> curStdDevs = kSingleTagStdDevs;

    // Simulation
    private PhotonCameraSim cameraSim;
    private VisionSystemSim visionSim;

    public Vision() {
        frontCamera = new PhotonCamera(frontCameraName);
        
        // Create PhotonPoseEstimator with MULTI_TAG_PNP_ON_COPROCESSOR as primary strategy
        frontCamPhotonEstimator = new PhotonPoseEstimator(
            kTagLayout, 
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            robotToFrontCam
        );
        
        // Set fallback strategy
        frontCamPhotonEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

        // ----- Simulation ------
        if (Robot.isSimulation()) {
            // Create the vision system simulation which handles cameras and targets on the field.
            visionSim = new VisionSystemSim("main");
            // Add all the AprilTags inside the tag layout as visible targets to this simulated field.
            visionSim.addAprilTags(kTagLayout);
            // Create simulated camera properties. These can be set to mimic your actual camera.
            var cameraProp = new SimCameraProperties();
            cameraProp.setCalibration(960, 720, Rotation2d.fromDegrees(90));
            cameraProp.setCalibError(0.35, 0.10);
            cameraProp.setFPS(15);
            cameraProp.setAvgLatencyMs(50);
            cameraProp.setLatencyStdDevMs(15);
            // Create a PhotonCameraSim which will update the linked PhotonCamera's values with visible
            // targets.
            cameraSim = new PhotonCameraSim(frontCamera, cameraProp);
            // Add the simulated camera to view the targets on this simulated field.
            visionSim.addCamera(cameraSim, robotToFrontCam);

            cameraSim.enableDrawWireframe(true);
        }
    }

    /**
     * Get the estimated robot pose from the front camera.
     * This method processes all unread results and returns the most recent estimate.
     * 
     * @return Optional containing the estimated robot pose, or empty if no valid estimate
     */
    public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
        var result = frontCamera.getLatestResult();
        
        // Only process if we have targets
        if (!result.hasTargets()) {
            curStdDevs = kSingleTagStdDevs;
            return Optional.empty();
        }
        
        // Update the pose estimator with the latest result
        Optional<EstimatedRobotPose> visionEst = frontCamPhotonEstimator.update(result);
        
        // Update standard deviations based on the estimate quality
        updateEstimationStdDevs(visionEst, result.getTargets());
        
        // Debug output
        if (visionEst.isPresent()) {
            SmartDashboard.putString("Vision/EstimatedPose", 
                String.format("(%.2f, %.2f, %.2f°)", 
                    visionEst.get().estimatedPose.getX(),
                    visionEst.get().estimatedPose.getY(),
                    visionEst.get().estimatedPose.getRotation().toRotation2d().getDegrees()));
            SmartDashboard.putNumber("Vision/NumTargets", result.getTargets().size());
            SmartDashboard.putNumber("Vision/Timestamp", visionEst.get().timestampSeconds);
        } else {
            SmartDashboard.putString("Vision/EstimatedPose", "No Estimate");
        }
        
        return visionEst;
    }

    /**
     * Calculates new standard deviations. This algorithm is a heuristic that creates dynamic standard
     * deviations based on number of tags, estimation strategy, and distance from the tags.
     *
     * @param estimatedPose The estimated pose to guess standard deviations for.
     * @param targets All targets in this camera frame
     */
    private void updateEstimationStdDevs(
            Optional<EstimatedRobotPose> estimatedPose, 
            List<PhotonTrackedTarget> targets) {
        
        if (estimatedPose.isEmpty()) {
            // No pose input. Default to single-tag std devs
            curStdDevs = kSingleTagStdDevs;
            SmartDashboard.putString("Vision/StdDevs", "Single Tag Default");
            return;
        }

        // Pose present. Start running Heuristic
        var estStdDevs = kSingleTagStdDevs;
        int numTags = 0;
        double avgDist = 0;

        // Precalculation - see how many tags we found, and calculate an average-distance metric
        for (var tgt : targets) {
            var tagPose = frontCamPhotonEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
            if (tagPose.isEmpty()) continue;
            numTags++;
            avgDist += tagPose.get().toPose2d().getTranslation()
                    .getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());
        }

        if (numTags == 0) {
            // No valid tags visible. Default to single-tag std devs
            curStdDevs = kSingleTagStdDevs;
            SmartDashboard.putString("Vision/StdDevs", "No Valid Tags");
        } else {
            // One or more tags visible, run the full heuristic.
            avgDist /= numTags;
            
            // Decrease std devs if multiple targets are visible
            if (numTags > 1) {
                estStdDevs = kMultiTagStdDevs;
            }
            
            // Increase std devs based on (average) distance
            // If single tag and far away, reject the measurement
            if (numTags == 1 && avgDist > 4) {
                estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
                SmartDashboard.putString("Vision/StdDevs", "Rejected - Too Far");
            } else {
                // Scale std devs based on distance
                estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
                SmartDashboard.putString("Vision/StdDevs", 
                    String.format("%d tags, %.2fm avg dist", numTags, avgDist));
            }
            
            curStdDevs = estStdDevs;
        }
        
        // Output the actual std dev values for debugging
        SmartDashboard.putNumber("Vision/StdDev_X", curStdDevs.get(0, 0));
        SmartDashboard.putNumber("Vision/StdDev_Y", curStdDevs.get(1, 0));
        SmartDashboard.putNumber("Vision/StdDev_Theta", curStdDevs.get(2, 0));
    }

    /**
     * Returns the latest standard deviations of the estimated pose, for use with
     * SwerveDrivePoseEstimator. This should only be used when there are targets visible.
     */
    public Matrix<N3, N1> getEstimationStdDevs() {
        return curStdDevs;
    }

    // ----- Simulation -----

    public void simulationPeriodic(Pose2d robotSimPose) {
        if (Robot.isSimulation() && visionSim != null) {
            visionSim.update(robotSimPose);
        }
    }

    /** Reset pose history of the robot in the vision system simulation. */
    public void resetSimPose(Pose2d pose) {
        if (Robot.isSimulation() && visionSim != null) {
            visionSim.resetRobotPose(pose);
        }
    }

    /** A Field2d for visualizing our robot and objects on the field. */
    public Field2d getSimDebugField() {
        if (!Robot.isSimulation() || visionSim == null) return null;
        return visionSim.getDebugField();
    }
}
