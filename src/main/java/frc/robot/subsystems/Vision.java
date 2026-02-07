package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.Constants.VisionConstants;

public class Vision extends SubsystemBase{

    private final PhotonCamera bottomCamera = new PhotonCamera(Constants.VisionConstants.kBottomCameraName);
    private final PhotonCamera topCamera = new PhotonCamera(VisionConstants.kRightCameraName);

    private final PhotonPoseEstimator bottomPhotonPoseEstimator;
    private final PhotonPoseEstimator topPhotonPoseEstimator;
    private Matrix<N3, N1> bottomCurStdDevs = VisionConstants.kSingleTagStdDevs;
    private Matrix<N3, N1> topCurStdDevs = VisionConstants.kSingleTagStdDevs;

    private Transform3d targetData;
    
    private final double[] array = {-0.03, 0.03};
    private CommandXboxController cont;

    private double horizVals;
    private double rotVals;

    private double pidVal;

    private boolean enablePoseEst = true;

    public Vision(CommandXboxController drivController) {
        this.cont = drivController;
        
        bottomCamera.setPipelineIndex(0);
        topCamera.setPipelineIndex(0);

        bottomPhotonPoseEstimator =
                new PhotonPoseEstimator(VisionConstants.kTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, VisionConstants.kBottomRobotToCam);
        bottomPhotonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY); 
        
        topPhotonPoseEstimator =
                new PhotonPoseEstimator(VisionConstants.kTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, VisionConstants.kTopRobotToCam);
        bottomPhotonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);  
    }

    public boolean targetDetected() {
        return targetData != null;
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Pose est enabled", enablePoseEst);
        
    }

    public Command togglePoseEst() {
        return this.runOnce(() -> {
           enablePoseEst = !enablePoseEst;
        });
     }
 
     /**
      * The latest estimated robot pose on the field from vision data. This may be empty. This should
      * only be called once per loop.
      *
      * <p>Also includes updates for the standard deviations, which can (optionally) be retrieved with
      * {@link getEstimationStdDevs}
      *
      * @return An {@link EstimatedRobotPose} with an estimated pose, estimate timestamp, and targets
      *     used for estimation.
      */
     public Optional<EstimatedRobotPose> getBottomCameraEstimatedGlobalPose() {
         Optional<EstimatedRobotPose> visionEst = Optional.empty();
         if (enablePoseEst) {
            for (var change : bottomCamera.getAllUnreadResults()) {
                visionEst = bottomPhotonPoseEstimator.update(change);
                bottomCurStdDevs = updateEstimationStdDevs(visionEst, change.getTargets(), bottomPhotonPoseEstimator);
             }
         }        
         return visionEst;
     }

     public Optional<EstimatedRobotPose> getTopCameraEstimatedGlobalPose() {
        Optional<EstimatedRobotPose> visionEst = Optional.empty();
        if (enablePoseEst) {
            for (var change : topCamera.getAllUnreadResults()) {
                visionEst = topPhotonPoseEstimator.update(change);
                topCurStdDevs = updateEstimationStdDevs(visionEst, change.getTargets(), topPhotonPoseEstimator);
             }
        }      
        return visionEst;
    }
 
     /**
      * Calculates new standard deviations This algorithm is a heuristic that creates dynamic standard
      * deviations based on number of tags, estimation strategy, and distance from the tags.
      *
      * @param estimatedPose The estimated pose to guess standard deviations for.
      * @param targets All targets in this camera frame
      */
     private Matrix<N3, N1> updateEstimationStdDevs(
             Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets, PhotonPoseEstimator poseEstimator) {
         Matrix<N3, N1> stdDevs = VisionConstants.kSingleTagStdDevs;
         if (estimatedPose.isEmpty()) {
             // No pose input. Default to single-tag std devs
             stdDevs = VisionConstants.kSingleTagStdDevs;
            
         } else {
             // Pose present. Start running Heuristic
             var estStdDevs = VisionConstants.kSingleTagStdDevs;
             int numTags = 0;
             double avgDist = 0;
 
             // Precalculation - see how many tags we found, and calculate an average-distance metric
             for (var tgt : targets) {
                 var tagPose = poseEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
                 if (tagPose.isEmpty()) continue;
                 numTags++;
                 avgDist +=
                         tagPose
                                 .get()
                                 .toPose2d()
                                 .getTranslation()
                                 .getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());
             }
 
             if (numTags == 0) {
                 // No tags visible. Default to single-tag std devs
                 stdDevs = VisionConstants.kSingleTagStdDevs;
             } else {
                 // One or more tags visible, run the full heuristic.
                 avgDist /= numTags;
                 // Decrease std devs if multiple targets are visible
                 if (numTags > 1) estStdDevs = VisionConstants.kMultiTagStdDevs;
                 // Increase std devs based on (average) distance
                 if (numTags == 1 && avgDist > 4)
                     estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
                 else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
                 stdDevs = estStdDevs;
             }
         }
         return stdDevs;
     }

     
 
     /**
      * Returns the latest standard deviations of the estimated pose from {@link
      * #getBottomCameraEstimatedGlobalPose()}, for use with {@link
      * edu.wpi.first.math.estimator.SwerveDrivePoseEstimator SwerveDrivePoseEstimator}. This should
      * only be used when there are targets visible.
      */
     public Matrix<N3, N1> getBottomEstimationStdDevs() {
         return bottomCurStdDevs;
     }

     public Matrix<N3, N1> getTopEstimationStdDevs() {
        return topCurStdDevs;
    }

}