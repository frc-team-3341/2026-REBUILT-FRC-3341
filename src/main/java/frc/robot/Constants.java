// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;
import com.pathplanner.lib.config.PIDConstants;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.DriveConstants.kDriveKinematics;
import static frc.robot.Constants.DriveConstants.kRobotMassKg;
import static frc.robot.Constants.DriveConstants.kWheelCOF;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;//imports for driveconstants(cause AYAAN DONT WANNA MAKE A DRIVE FOLDER :>D)
import edu.wpi.first.math.system.plant.DCMotor;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class ModeConstants {
    public static final Mode simMode = Mode.SIM;
    public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

    public enum Mode {
      REAL,
      SIM, // physics sim
      REPLAY // replaying fro log files
    }
  }

  public static final class DriveConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 4.8;
    public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second
    public static final double odometryFrequency = 100.0; // Hz
    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(27.0);
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(27.0);
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    public static final double driveBaseRadius = Math.hypot(kTrackWidth / 2.0, kWheelBase / 2.0);

    // Angular offsets of the modules relative to the chassis in radians
    private static final double kEasySwerveAngularOffsetCompensation = Math.PI / 4;
    public static final double kFrontLeftChassisAngularOffset = 0;
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kRearLeftChassisAngularOffset = 0;
    public static final double kRearRightChassisAngularOffset = 0;

    // The EasySwerve module allows installation of the motors either on top or
    // bottom of the module.
    // These constants configure the location of the motors. The default
    // configuration is with both
    // motors on the bottom of the module.
    public static final boolean kFrontLeftDrivingMotorOnBottom = false;
    public static final boolean kRearLeftDrivingMotorOnBottom = true;
    public static final boolean kFrontRightDrivingMotorOnBottom = false;
    public static final boolean kRearRightDrivingMotorOnBottom = false;

    public static final boolean kFrontLeftTurningMotorOnBottom = false;
    public static final boolean kRearLeftTurningMotorOnBottom = false;
    public static final boolean kFrontRightTurningMotorOnBottom = false;
    public static final boolean kRearRightTurningMotorOnBottom = false;

    // SPARK MAX CAN IDs
    public static final int kFrontLeftDrivingCanId = 1;
    public static final int kRearLeftDrivingCanId = 5;
    public static final int kFrontRightDrivingCanId = 3;
    public static final int kRearRightDrivingCanId = 7;

    public static final int kFrontLeftTurningCanId = 2;
    public static final int kRearLeftTurningCanId = 6;
    public static final int kFrontRightTurningCanId = 4;
    public static final int kRearRightTurningCanId = 8;

    public static final boolean kGyroReversed = false;
    public static final Rotation2d navxOffset = Rotation2d.fromDegrees(-90); // Offsets driving by 90 degrees clockwise

    public static final double kRobotMassKg = 54.43; // TODO: Adjust to your robot's mass
    public static final double kWheelCOF = 1.2; // TODO: Coefficient of friction (from pathplanner settings)

    public static final int drivingCurrentLimit = 40; // amps
    public static final int turningCurrentLimit = 20;

    public static final PIDConstants turningPID = new PIDConstants(1, 0, 0);
    public static final PIDConstants drivingPID = new PIDConstants(0.04, 0, 0);

    // TODO: CONSTANTS TAKEN FROM HERE
    // https://github.com/Shenzhen-Robotics-Alliance/AdvantageKit-SparkSwerveTemplate-MapleSim/blob/9b988399e73c70c1efdf33513f3183908f242504/src/main/java/frc/robot/subsystems/drive/DriveConstants.java
    // We need to determine our own k values for the robot (i think leaving the sim
    // ones are fine)
    public static final double driveKp = 0.0;
    public static final double driveKd = 0.0;
    public static final double driveKs = 0.0;
    public static final double driveKv = 0.1;
    public static final double driveSimP = 0.05;
    public static final double driveSimD = 0.0;
    public static final double driveSimKs = 0.0;
    public static final double driveSimKv = 0.0789;
    public static final double turnPIDMinInput = 0; // Radians
    public static final double turnPIDMaxInput = 2 * Math.PI; // Radians
    // I think these two above are the same as the outputrange lines in config.java

  }

  public static class Dimensions {
    public static final Distance BUMPER_THICKNESS = Inches.of(3); // frame to edge of bumper
    public static final Distance BUMPER_HEIGHT = Inches.of(7); // height from floor to top of bumper
    public static final Distance FRAME_SIZE_Y = Inches.of(26.25); // left to right (y-axis)
    public static final Distance FRAME_SIZE_X = Inches.of(28.75); // front to back (x-axis)

    public static final Distance FULL_WIDTH = FRAME_SIZE_Y.plus(BUMPER_THICKNESS.times(2));
    public static final Distance FULL_LENGTH = FRAME_SIZE_X.plus(BUMPER_THICKNESS.times(2));
    // Currently in pathplanner settings can change later
    // public static final RobotConfig ppConfig = new RobotConfig(
    // kRobotMassKg,
    // robotMOI,
    // new ModuleConfig(
    // wheelRadiusMeters,
    // maxSpeedMetersPerSec,
    // wheelCOF,
    // driveGearbox.withReduction(driveMotorReduction),
    // driveMotorCurrentLimit,
    // 1),
    // kDriveKinematics);
    public static final DriveTrainSimulationConfig mapleSimConfig = DriveTrainSimulationConfig.Default()
        .withCustomModuleTranslations(kDriveKinematics.getModules())
        .withRobotMass(Kilogram.of(kRobotMassKg))
        .withGyro(COTS.ofPigeon2())
        .withSwerveModule(new SwerveModuleSimulationConfig(
            DCMotor.getNeoVortex(1),
            DCMotor.getNeoVortex(1),
            ModuleConstants.kDrivingMotorReduction,
            ModuleConstants.kDrivingMotorReduction,
            Volts.of(0.1),
            Volts.of(0.1),
            Meters.of(ModuleConstants.kWheelDiameterMeters / 2),
            KilogramSquareMeters.of(0.02),
            kWheelCOF));
  }
  // public static final class VisionConstants {
  // public static final String frontCameraName = "pterodactyl";
  // // See
  // https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html#robot-coordinate-system
  // // for why these values the way they are. In short x is positive towards the
  // front, y is positive to left, z is positive to the sky
  // private static final double frontCamPitch = Units.degreesToRadians(25.0);
  // //TODO: for testing, check what the camera pitch is
  // public static final Transform3d robotToFrontCam =
  // new Transform3d(new Translation3d(Units.inchesToMeters(12.75),
  // Units.inchesToMeters(0), Units.inchesToMeters(12.5)), new Rotation3d(0,
  // frontCamPitch, 0));

  // // The layout of the AprilTags on the field
  // public static final AprilTagFieldLayout kTagLayout =
  // AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

  // // The standard deviations of our vision estimated poses, which affect
  // correction rate
  // // TODO: (Fake values. Experiment and determine estimation noise on an actual
  // robot.)
  // public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4,
  // 8);
  // public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5,
  // 0.5, 1);

  // }

  public static class VisionConstants {
    // Standard deviation baselines for 1 meter distance to single tag
    public static final double LINEAR_STD_DEV_BASELINE = 0.08; // Meters
    public static final double ANGULAR_STD_DEV_BASELINE = 1.0; // Radians

    public static final String[] CAMERA_NAMES = {
        "pterodactyl"
    };

    public static final double MAX_AMBIGUITY = 0.3;

    public static final AprilTagFieldLayout APRIL_TAGS = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

    // Transforms from robot to cameras, (x forward, y left, z up), (roll, pitch,
    // yaw)
    // TODO: Add vision constants for position and rotation (each camera has its
    // own)
    public static final Transform3d[] CAMERA_TRANSFORMS = {
        new Transform3d(new Translation3d(), new Rotation3d())
    };
  }

  public static class ModuleConstants {
    // The EasySwerve module can only be configured with one pinion gears: 12T.
    public static final int kDrivingMotorPinionTeeth = 12;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = Units.inchesToMeters(4);
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 30 teeth on the first-stage spur gear,
    // 15 teeth on the bevel pinion
    public static final double kDrivingWheelBevelGearTeeth = 45.0;
    public static final double kDrivingWheelFirstStageSpurGearTeeth = 30.0;
    public static final double kDrivingMotorBevelPinionTeeth = 15.0;
    public static final double kDrivingMotorReduction = (kDrivingWheelBevelGearTeeth
        * kDrivingWheelFirstStageSpurGearTeeth)
        / (kDrivingMotorPinionTeeth * kDrivingMotorBevelPinionTeeth);
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
        / kDrivingMotorReduction;
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kMechJoystickPort = 1;
    public static final double kDriveDeadband = 0.1;
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }

  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
  }

  public static final class ShooterConstants {
    public static final double hubHeight = 1.43; 
  }

  public static final class FieldConstants {
    public static final Pose2d blueHubCenterPose = new Pose2d(4.633, 4.035, new Rotation2d(0));
    public static final Pose2d redHubCenterPose = new Pose2d(11.907, 4.035, new Rotation2d(0));
  }
}