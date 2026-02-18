// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.vision;

import static frc.robot.Constants.VisionConstants.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import java.util.function.Supplier;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

/** IO implementation for physics sim using PhotonVision simulator. */
public class VisionIOPhotonVisionSim extends VisionIOPhotonVision {
    private static VisionSystemSim visionSim;

    private final Supplier<Pose2d> poseSupplier;
    private final PhotonCameraSim cameraSim;

    /**
     * Creates a new VisionIOPhotonVisionSim.
     *
     * @param name           The name of the camera.
     * @param robotToCamera  The 3D transform from robot center to camera.
     * @param poseSupplier   Supplier for the ground-truth simulated robot pose
     *                       (from maple-sim, NOT the odometry estimator).
     */
    public VisionIOPhotonVisionSim(String name, Transform3d robotToCamera, Supplier<Pose2d> poseSupplier) {
        super(name, robotToCamera);
        this.poseSupplier = poseSupplier;

        // Initialize the shared VisionSystemSim once across all cameras.
        // PhotonLib auto-publishes a Field2d to NT at:
        //   /VisionSystemSim-main/Sim Field
        // AdvantageScope can read this directly for the vision camera visualization.
        if (visionSim == null) {
            visionSim = new VisionSystemSim("main");
            visionSim.addAprilTags(APRIL_TAGS);
        }

        // IMPORTANT: Configure SimCameraProperties BEFORE passing them to
        // PhotonCameraSim. Properties set after construction are ignored.
        var cameraProperties = new SimCameraProperties();
        // 960x720 resolution, 90° diagonal FOV — adjust to match your camera calibration.
        cameraProperties.setCalibration(960, 720, Rotation2d.fromDegrees(90));
        // Simulated pixel-detection noise (average error px, std-dev px).
        cameraProperties.setCalibError(0.25, 0.08);
        // Simulated camera framerate (capped by robot loop rate in sim).
        cameraProperties.setFPS(20);
        // Simulated pipeline latency (avg ms, std-dev ms).
        cameraProperties.setAvgLatencyMs(35);
        cameraProperties.setLatencyStdDevMs(5);

        // Create the simulated camera, linked to the real PhotonCamera so it
        // spoofs the same NetworkTables topics a real camera publishes.
        cameraSim = new PhotonCameraSim(camera, cameraProperties);

        // Enable NT video streams. Required for AdvantageScope to see the
        // camera feed and for the WPILib sim GUI camera viewer to work.
        cameraSim.enableRawStream(true);
        cameraSim.enableProcessedStream(true);
        // Wireframe rendering is very CPU-heavy; leave disabled unless debugging visually.
        // cameraSim.enableDrawWireframe(true);

        // Register this camera in the shared vision world.
        visionSim.addCamera(cameraSim, robotToCamera);
    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {
        // Feed the ground-truth maple-sim pose into the vision world each loop.
        // Using the odometry-estimated pose here would create a feedback loop.
        visionSim.update(poseSupplier.get());
        super.updateInputs(inputs);
    }
}