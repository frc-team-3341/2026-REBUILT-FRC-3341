package frc.robot.subsystems;
import static frc.robot.Constants.VisionConstants.*;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
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
    private final PhotonCamera intakeCamera;
    private final PhotonCamera batteryCamera;
    private final PhotonCamera shooterCamera;

    private PhotonCamera[] cameras;

    private final PhotonPoseEstimator intakePhotonPoseEstimator;
    private final PhotonPoseEstimator batteryPhotonPoseEstimator;
    private final PhotonPoseEstimator shooterPhotonPoseEstimator;
    private Matrix<N3, N1> curStdDevs = kSingleTagStdDevs;

    private final StructPublisher<Pose2d> intakePoseEstimatorPublisher;
    private final StructPublisher<Pose2d> shooterPoseEstimatorPublisher;
    private final StructPublisher<Pose2d> batteryPoseEstimatorPublisher;

    // Simulation
    private PhotonCameraSim cameraSim;
    private VisionSystemSim visionSim;

    public Vision() {
        intakeCamera = new PhotonCamera(intakeCameraName);
        batteryCamera = new PhotonCamera(batteryCameraName);
        shooterCamera = new PhotonCamera(shooterCameraName);
        
        cameras = new PhotonCamera[] {intakeCamera, batteryCamera, shooterCamera};

        // Create PhotonPoseEstimator with MULTI_TAG_PNP_ON_COPROCESSOR as primary strategy
        intakePhotonPoseEstimator = new PhotonPoseEstimator(
            kTagLayout, 
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            robotToIntakeCam
        );
        batteryPhotonPoseEstimator = new PhotonPoseEstimator(
            kTagLayout, 
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            robotToBatteryCam
        );
        shooterPhotonPoseEstimator = new PhotonPoseEstimator(
            kTagLayout, 
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            robotToShooterCam
        );
    
        // Set fallback strategy
        intakePhotonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

        batteryPhotonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
        
        shooterPhotonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

        intakePoseEstimatorPublisher = NetworkTableInstance.getDefault()
        .getStructTopic("/PoseEstimator/IntakePose", Pose2d.struct).publish();

        shooterPoseEstimatorPublisher = NetworkTableInstance.getDefault()
        .getStructTopic("/PoseEstimator/ShooterPose", Pose2d.struct).publish();

        batteryPoseEstimatorPublisher = NetworkTableInstance.getDefault()
        .getStructTopic("/PoseEstimator/BatteryPose", Pose2d.struct).publish();

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
            cameraSim = new PhotonCameraSim(intakeCamera, cameraProp);
            // Add the simulated camera to view the targets on this simulated field.
            visionSim.addCamera(cameraSim, robotToIntakeCam);

            cameraSim.enableDrawWireframe(true);
        }
    }

        /**
     * Get the estimated robot pose from the intake camera.
     * This method processes all unread results and returns the most recent estimate.
     * 
     * @return Optional containing the estimated robot pose, or empty if no valid estimate
     */
    public Optional<EstimatedRobotPose> getIntakeEstimatedGlobalPose() {
        
        // var result = intakeCamera.getLatestResult();
        Optional<EstimatedRobotPose> visionEst = Optional.empty();

        for (var result : intakeCamera.getAllUnreadResults()) {
            // // Only process if we have targets
            // if (!result.hasTargets()) {
            // curStdDevs = kSingleTagStdDevs;
            // return Optional.empty();
            // }
            
            // Update the pose estimator with the latest result
            visionEst = intakePhotonPoseEstimator.update(result);
        
            // Update standard deviations based on the estimate quality
            updateEstimationStdDevs(visionEst, result.getTargets());

            if (visionEst.isPresent()) {
            intakePoseEstimatorPublisher.set(new Pose2d(visionEst.get().estimatedPose.getX(), 
                visionEst.get().estimatedPose.getY(), new Rotation2d(visionEst.get().estimatedPose.getRotation().getMeasureZ())));
            }
            
        }
        

        
        // // Debug output
        // if (visionEst.isPresent()) {
        //     SmartDashboard.putString("Vision/EstimatedPose", 
        //         String.format("(%.2f, %.2f, %.2f°)", 
        //             visionEst.get().estimatedPose.getX(),
        //             visionEst.get().estimatedPose.getY(),
        //             visionEst.get().estimatedPose.getRotation().toRotation2d().getDegrees()));
        //     SmartDashboard.putNumber("Vision/NumTargets", result.getTargets().size());
        //     SmartDashboard.putNumber("Vision/Timestamp", visionEst.get().timestampSeconds);
        // } else {
        //     SmartDashboard.putString("Vision/EstimatedPose", "No Estimate");
        // }
        
        return visionEst;
    }
        /**
     * Get the estimated robot pose from the battery camera.
     * This method processes all unread results and returns the most recent estimate.
     * 
     * @return Optional containing the estimated robot pose, or empty if no valid estimate
     */
    public Optional<EstimatedRobotPose> getBatteryEstimatedGlobalPose() {
        
        // var result = intakeCamera.getLatestResult();
        Optional<EstimatedRobotPose> visionEst = Optional.empty();

        for (var result : batteryCamera.getAllUnreadResults()) {
            // // Only process if we have targets
            // if (!result.hasTargets()) {
            // curStdDevs = kSingleTagStdDevs;
            // return Optional.empty();
            // }

            // Update the pose estimator with the latest result
            visionEst = batteryPhotonPoseEstimator.update(result);
        
            // Update standard deviations based on the estimate quality
            updateEstimationStdDevs(visionEst, result.getTargets());

            if (visionEst.isPresent()) {
            batteryPoseEstimatorPublisher.set(new Pose2d(visionEst.get().estimatedPose.getX(), 
                visionEst.get().estimatedPose.getY(), new Rotation2d(visionEst.get().estimatedPose.getRotation().getMeasureZ())));
            }
        }
        

        
        // // Debug output
        // if (visionEst.isPresent()) {
        //     SmartDashboard.putString("Vision/EstimatedPose", 
        //         String.format("(%.2f, %.2f, %.2f°)", 
        //             visionEst.get().estimatedPose.getX(),
        //             visionEst.get().estimatedPose.getY(),
        //             visionEst.get().estimatedPose.getRotation().toRotation2d().getDegrees()));
        //     SmartDashboard.putNumber("Vision/NumTargets", result.getTargets().size());
        //     SmartDashboard.putNumber("Vision/Timestamp", visionEst.get().timestampSeconds);
        // } else {
        //     SmartDashboard.putString("Vision/EstimatedPose", "No Estimate");
        // }
        
        return visionEst;
    }
        /**
     * Get the estimated robot pose from the shooter camera.
     * This method processes all unread results and returns the most recent estimate.
     * 
     * @return Optional containing the estimated robot pose, or empty if no valid estimate
     */
    public Optional<EstimatedRobotPose> getShooterEstimatedGlobalPose() {
        
        // var result = intakeCamera.getLatestResult();
        Optional<EstimatedRobotPose> visionEst = Optional.empty();

        for (var result : shooterCamera.getAllUnreadResults()) {
            // // Only process if we have targets
            // if (!result.hasTargets()) {
            // curStdDevs = kSingleTagStdDevs;
            // return Optional.empty();
            // }
            
            // Update the pose estimator with the latest result
            visionEst = shooterPhotonPoseEstimator.update(result);
        
            // Update standard deviations based on the estimate quality
            updateEstimationStdDevs(visionEst, result.getTargets());

            if (visionEst.isPresent()) {
                shooterPoseEstimatorPublisher.set(new Pose2d(visionEst.get().estimatedPose.getX(), 
                visionEst.get().estimatedPose.getY(), new Rotation2d(visionEst.get().estimatedPose.getRotation().getMeasureZ())));
            }
            
        }
        

        
        // // Debug output
        // if (visionEst.isPresent()) {
        //     SmartDashboard.putString("Vision/EstimatedPose", 
        //         String.format("(%.2f, %.2f, %.2f°)", 
        //             visionEst.get().estimatedPose.getX(),
        //             visionEst.get().estimatedPose.getY(),
        //             visionEst.get().estimatedPose.getRotation().toRotation2d().getDegrees()));
        //     SmartDashboard.putNumber("Vision/NumTargets", result.getTargets().size());
        //     SmartDashboard.putNumber("Vision/Timestamp", visionEst.get().timestampSeconds);
        // } else {
        //     SmartDashboard.putString("Vision/EstimatedPose", "No Estimate");
        // }
        
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
            var tagPose = intakePhotonPoseEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
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
            if (numTags == 1 && avgDist > 2.5) {
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
    /**
     * Get location of hub apriltag relative to the shooter camera 
     * @return Optional containing a transform3d of the apriltag on the hub
     */
    public Optional<Transform3d> getHubApriltagLocation() {
        var results = shooterCamera.getLatestResult();
        if (results.hasTargets()) {
            for (var tgt: results.getTargets()) {
                int id = tgt.getFiducialId();
                // Apriltags of hubs: 9,10,25,26
                // The right apriltag should be enough to figure out if you are inline
                if (id == 9) {
                    Transform3d targetTransform = tgt.getBestCameraToTarget();
                    return Optional.of(targetTransform);
                }
                if (id == 26) {
                    Transform3d targetTransform = tgt.getBestCameraToTarget();
                    return Optional.of(targetTransform);
                }
            }
        }
        return Optional.empty();
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