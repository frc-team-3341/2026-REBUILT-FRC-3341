// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems;

import frc.robot.subsystems.Modules.*;
import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.DriveConstants.kDriveKinematics;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.ModeConstants;
import frc.robot.Constants.ShooterConstants;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.Superstructure.SwerveState;
import frc.robot.subsystems.Gyro.GyroIO;
import static frc.util.ClimberUtil.getTowerPoses;
import frc.robot.Constants.AutoConstants;
import frc.util.LocalADStarAK;
import frc.util.ShooterUtil;

import java.util.Map;
import java.util.Optional;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.Consumer;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import frc.robot.subsystems.Gyro.GyroIOInputsAutoLogged;

public class Drive extends SubsystemBase implements Vision.VisionConsumer {
    static final Lock odometryLock = new ReentrantLock();
    private final GyroIO gyroIO;
    private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
    private final EasySwerveModule[] modules = new EasySwerveModule[4]; // FL, FR, BL, BR
    private final SysIdRoutine sysId;
    private ChassisSpeeds currSpeeds;
    private boolean aimDriveEnabled;

    private PIDController aimDriveController = new PIDController(0.1, 0, 0.05);

    private final Alert gyroDisconnectedAlert = new Alert("Disconnected gyro, using kinematics as fallback.",
            AlertType.kError);

    private Rotation2d rawGyroRotation = new Rotation2d();
    private final SwerveModulePosition[] lastModulePositions = // For delta tracking
            new SwerveModulePosition[] {
                    new SwerveModulePosition(),
                    new SwerveModulePosition(),
                    new SwerveModulePosition(),
                    new SwerveModulePosition()
            };
    private final SwerveDrivePoseEstimator poseEstimator = new SwerveDrivePoseEstimator(DriveConstants.kDriveKinematics,
            rawGyroRotation, lastModulePositions, new Pose2d());
    private final Consumer<Pose2d> resetSimulationPoseCallBack;

    public Drive(
            GyroIO gyroIO,
            EasySwerveModuleIO flModuleIO,
            EasySwerveModuleIO frModuleIO,
            EasySwerveModuleIO blModuleIO,
            EasySwerveModuleIO brModuleIO,
            Consumer<Pose2d> resetSimulationPoseCallBack) {
        this.gyroIO = gyroIO;
        this.resetSimulationPoseCallBack = resetSimulationPoseCallBack;
        modules[0] = new EasySwerveModule(flModuleIO, 0);
        modules[1] = new EasySwerveModule(frModuleIO, 1);
        modules[2] = new EasySwerveModule(blModuleIO, 2);
        modules[3] = new EasySwerveModule(brModuleIO, 3);

        //this is required to stop the robot from tweaking when going from -179 to 179
        aimDriveController.enableContinuousInput(-180, 180);

        //Set tolerance to prevent pid controller oscillation
        aimDriveController.setTolerance(1.0);

        // Usage reporting for swerve template
        // HAL.report(tResourceType.kResourceType_RobotDrive,
        // tInstances.kRobotDriveSwerve_AdvantageKit);
        
        aimDriveEnabled = false;

        // Start odometry thread
        SparkOdometryThread.getInstance().start();
        try {
            RobotConfig config = RobotConfig.fromGUISettings();
            // Configure AutoBuilder for PathPlanner
            AutoBuilder.configure(
                    this::getPose,
                    this::resetOdometry,
                    this::getChassisSpeeds,
                    this::runVelocity,
                    new PPHolonomicDriveController(new PIDConstants(5.0, 0.0, 0.0), new PIDConstants(5.0, 0.0, 0.0)),
                    config,
                    () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
                    this);
            Pathfinding.setPathfinder(new LocalADStarAK());
            PathPlannerLogging.setLogActivePathCallback((activePath) -> {
                Logger.recordOutput("Odometry/Trajectory", activePath.toArray(new Pose2d[activePath.size()]));
            });
            PathPlannerLogging.setLogTargetPoseCallback((targetPose) -> {
                Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose);
            });

        } catch (Exception e) {
            e.printStackTrace();
            System.out.println("uh oh auto is broken!!");
        }

        // Configure SysId
        sysId = new SysIdRoutine(
                new SysIdRoutine.Config(
                        null, null, null, (state) -> Logger.recordOutput("Drive/SysIdState", state.toString())),
                new SysIdRoutine.Mechanism((voltage) -> runCharacterization(voltage.in(Volts)), null, this));
    }


    @Override
    public void periodic() {
        odometryLock.lock(); // Prevents odometry updates while reading data
        gyroIO.updateInputs(gyroInputs);
        Logger.processInputs("Drive/Gyro", gyroInputs);
        for (var module : modules) {
            module.periodic();
        }
        odometryLock.unlock();

        // Stop moving when disabled
        if (DriverStation.isDisabled()) {
            for (var module : modules) {
                module.stop();
            }
        }

        // Log empty setpoint states when disabled
        if (DriverStation.isDisabled()) {
            Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[] {});
            Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});
        }

        Logger.recordOutput("distance to hub", ShooterUtil.getDistanceToHub(getPose()));
        Logger.recordOutput("launch velocity", ShooterUtil.calculateLinearLaunchVelocity(ShooterUtil.getDistanceToHub(getPose()),
                                        ShooterConstants.shooterHeight, 
                                        0));
        
        ShooterUtil.updateTrajectory(getPose(), ShooterConstants.shooterHeight);

        // Update odometry
        double[] sampleTimestamps = modules[0].getOdometryTimestamps(); // All signals are sampled together
        int sampleCount = sampleTimestamps.length;
        for (int i = 0; i < sampleCount; i++) {
            // Read wheel positions and deltas from each module
            SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
            SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
            for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
                modulePositions[moduleIndex] = modules[moduleIndex].getOdometryPositions()[i];
                moduleDeltas[moduleIndex] = new SwerveModulePosition(
                        modulePositions[moduleIndex].distanceMeters - lastModulePositions[moduleIndex].distanceMeters,
                        modulePositions[moduleIndex].angle);
                lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
            }

            // Update gyro angle
            if (gyroInputs.connected) {
                // Use the real gyro angle
                rawGyroRotation = gyroInputs.odometryYawPositions[i];
            } else {
                // Use the angle delta from the kinematics and module deltas
                Twist2d twist = DriveConstants.kDriveKinematics.toTwist2d(moduleDeltas);
                rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
            }

            // Apply update
            poseEstimator.updateWithTime(sampleTimestamps[i], rawGyroRotation, modulePositions);
        }

        // Update gyro alert
        gyroDisconnectedAlert
                .set(!gyroInputs.connected && Constants.ModeConstants.currentMode != ModeConstants.Mode.SIM);
    }

    /**
     * Runs the drive at the desired velocity.
     *
     * @param speeds Speeds in meters/sec
     */
    public void runVelocity(ChassisSpeeds speeds) {
        // Calculate module setpoints
        speeds = ChassisSpeeds.discretize(speeds, 0.02);
        SwerveModuleState[] setpointStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, DriveConstants.kMaxSpeedMetersPerSecond);
        currSpeeds = speeds;

        // Log unoptimized setpoints
        Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
        Logger.recordOutput("SwerveChassisSpeeds/Setpoints", speeds);

        // Send setpoints to modules
        for (int i = 0; i < 4; i++) {
            modules[i].runSetpoint(setpointStates[i]);
        }

        // Log optimized setpoints (runSetpoint mutates each state)
        Logger.recordOutput("SwerveStates/SetpointsOptimized", setpointStates);
    }

    public void drive(ChassisSpeeds speeds, boolean fieldRelative) {
        if (fieldRelative) {
            if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red) {
                speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                    speeds, getRotation().plus(new Rotation2d(Math.PI)));
            }
            else {
                speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, getRotation());
            }
        }
        
        speeds = ChassisSpeeds.discretize(speeds, 0.02);

        currSpeeds = speeds;

        SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, DriveConstants.kMaxSpeedMetersPerSecond);

        for (int i = 0; i < 4; i++) {
            modules[i].runSetpoint(moduleStates[i]);
        }
    }

    //broken for red alliance for now
    public void aimDrive(double xSpeed, double ySpeed) { 

      Pose2d currentPose = getPose();

      Translation2d hubCenterPose = ShooterUtil.getHubTranslation2d();

      //end method if there is no alliance selected cuz this should only be used on the field
      if (hubCenterPose == null) {
        return;
      }
      
      double theta;

      double xDisplacement = (hubCenterPose.getX() - currentPose.getX()); 
      double yDisplacement = (hubCenterPose.getY() - currentPose.getY());

      theta = Math.toDegrees(Math.atan2(yDisplacement, xDisplacement)); 

      Logger.recordOutput("theta offset", theta);
      Logger.recordOutput("x displacement", xDisplacement);
      Logger.recordOutput("y displacement", yDisplacement);
      
      double rotOutput = aimDriveController.calculate(getRotation().getDegrees(), theta); 

      drive(new ChassisSpeeds(xSpeed, ySpeed, rotOutput), true);
  }

    public ChassisSpeeds getFieldSpeeds() {
        if (currSpeeds == null)
            return new ChassisSpeeds();

        return ChassisSpeeds.fromRobotRelativeSpeeds(currSpeeds, getRotation());
    }

    /** Runs the drive in a straight line with the specified drive output. */
    public void runCharacterization(double output) {
        for (int i = 0; i < 4; i++) {
            modules[i].runCharacterization(output);
        }
    }

    //might want to move this into a separate file in the future
    public Command handleSwerveTransitions(SwerveState desiredState) {
        switch (desiredState) {
            case MANUAL:
                return this.runOnce(() -> aimDriveEnabled = false);

            case TRACKING_HUB:
                return this.runOnce(() -> aimDriveEnabled = true);

            case ALIGNING_TOWER_LEFT:
                return AutoBuilder.pathfindToPose(
                    getTowerPoses()[0], AutoConstants.PATH_CONSTRAINTS, 0);

            case ALIGNING_TOWER_RIGHT:
                return AutoBuilder.pathfindToPose(
                    getTowerPoses()[1], AutoConstants.PATH_CONSTRAINTS, 0);

            default:
                return Commands.print("Invalid Swerve State Provided!");
        }
    }

    /** Stops the drive. */
    public void stop() {
        runVelocity(new ChassisSpeeds());
    }

    public boolean aimDriveEnabled() {
        return aimDriveEnabled;
    }

    /**
     * Stops the drive and turns the modules to an X arrangement to resist movement.
     * The modules will return to their
     * normal orientations the next time a nonzero velocity is requested.
     */
    public void stopWithX() {
        Rotation2d[] headings = new Rotation2d[4];
        for (int i = 0; i < 4; i++) {
            headings[i] = DriveConstants.kDriveKinematics.getModules()[i].getAngle();
        }
        DriveConstants.kDriveKinematics.resetHeadings(headings);
        stop();
    }

    /** Returns a command to run a quasistatic test in the specified direction. */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return run(() -> runCharacterization(0.0)).withTimeout(1.0).andThen(sysId.quasistatic(direction));
    }

    /** Returns a command to run a dynamic test in the specified direction. */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return run(() -> runCharacterization(0.0)).withTimeout(1.0).andThen(sysId.dynamic(direction));
    }

    /**
     * Returns the module states (turn angles and drive velocities) for all of the
     * modules.
     */
    @AutoLogOutput(key = "SwerveStates/Measured")
    private SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (int i = 0; i < 4; i++) {
            states[i] = modules[i].getState();
        }
        return states;
    }

    /**
     * Returns the module positions (turn angles and drive positions) for all of the
     * modules.
     */
    private SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] states = new SwerveModulePosition[4];
        for (int i = 0; i < 4; i++) {
            states[i] = modules[i].getPosition();
        }
        return states;
    }

    /** Returns the measured chassis speeds of the robot. */
    @AutoLogOutput(key = "SwerveChassisSpeeds/Measured")
    private ChassisSpeeds getChassisSpeeds() {
        return DriveConstants.kDriveKinematics.toChassisSpeeds(getModuleStates());
    }

    /** Returns the position of each module in radians. */
    public double[] getWheelRadiusCharacterizationPositions() {
        double[] values = new double[4];
        for (int i = 0; i < 4; i++) {
            values[i] = modules[i].getWheelRadiusCharacterizationPosition();
        }
        return values;
    }

    /** Returns the average velocity of the modules in rad/sec. */
    public double getFFCharacterizationVelocity() {
        double output = 0.0;
        for (int i = 0; i < 4; i++) {
            output += modules[i].getFFCharacterizationVelocity() / 4.0;
        }
        return output;
    }

    /** Returns the current odometry pose. */
    @AutoLogOutput(key = "Odometry/Robot")
    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    /** Returns the current odometry rotation. */
    public Rotation2d getRotation() {
        return getPose().getRotation();
    }

    /** Resets the current odometry pose. */
    public void resetOdometry(Pose2d pose) {
        resetSimulationPoseCallBack.accept(pose);
        poseEstimator.resetPosition(rawGyroRotation, getModulePositions(), pose);
    }

    /** Adds a new timestamped vision measurement. */
    @Override
    public void accept(Pose2d visionRobotPoseMeters, double timestampSeconds, Matrix<N3, N1> visionMeasurementStdDevs) {
        poseEstimator.addVisionMeasurement(visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
    }

    /** Returns the maximum linear speed in meters per sec. */
    public double getMaxLinearSpeedMetersPerSec() {
        return DriveConstants.kMaxSpeedMetersPerSecond;

    }

    /** Returns the maximum angular speed in radians per sec. */
    public double getMaxAngularSpeedRadPerSec() {
        return DriveConstants.kMaxSpeedMetersPerSecond / DriveConstants.driveBaseRadius;
    }
}