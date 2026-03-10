// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
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
  public static final class DriveConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 4.8;
    public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second

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

    // Angular offsets of the modules relative to the chassis in radians
    private static final double kEasySwerveAngularOffsetCompensation = Math.PI / 4;
    public static final double kFrontLeftChassisAngularOffset = 0;
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kRearLeftChassisAngularOffset = 0;
    public static final double kRearRightChassisAngularOffset = 0;

    // The EasySwerve module allows installation of the motors either on top or bottom of the module.
    // These constants configure the location of the motors. The default configuration is with both
    // motors on the bottom of the module.
    public static final boolean kFrontLeftDrivingMotorOnBottom = true;
    public static final boolean kRearLeftDrivingMotorOnBottom = false;
    public static final boolean kFrontRightDrivingMotorOnBottom = false;
    public static final boolean kRearRightDrivingMotorOnBottom = true;

    public static final boolean kFrontLeftTurningMotorOnBottom = false;
    public static final boolean kRearLeftTurningMotorOnBottom = false;
    public static final boolean kFrontRightTurningMotorOnBottom = false;
    public static final boolean kRearRightTurningMotorOnBottom = false;

    // SPARK MAX CAN IDs
    //TODO: WE NEED TO CHANGE THESE BEFORE TESTING!!!!!
    public static final int kFrontLeftDrivingCanId = 1;
    public static final int kRearLeftDrivingCanId = 5;
    public static final int kFrontRightDrivingCanId = 3;
    public static final int kRearRightDrivingCanId = 7;

    public static final int kFrontLeftTurningCanId = 2;
    public static final int kRearLeftTurningCanId = 6;
    public static final int kFrontRightTurningCanId = 4;
    public static final int kRearRightTurningCanId = 8;


    public static final boolean kGyroReversed = false;
    public static final Rotation2d navxOffset = Rotation2d.fromDegrees(0); //Offsets driving by 90 degrees clockwise
    public static final int intakeMotorCanId = 10;

    public static final double kRobotMassKg = 54.43; // TODO: Adjust to your robot's mass
    public static final double kWheelCOF = 1.2; // TODO: Coefficient of friction (from pathplanner settings)

  }
      public static final class VisionConstants {
        public static final String frontCameraName = "pterodactyl";
        // See https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html#robot-coordinate-system
        // for why these values the way they are. In short x is positive towards the front, y is positive to left, z is positive to the sky
        private static final double frontCamPitch = Units.degreesToRadians(25.0); //TODO: for testing, check what the camera pitch is
        public static final Transform3d robotToFrontCam =
                new Transform3d(new Translation3d(Units.inchesToMeters(12.75), Units.inchesToMeters(0), Units.inchesToMeters(12.5)), new Rotation3d(0, frontCamPitch, 0));

        // The layout of the AprilTags on the field
        public static final AprilTagFieldLayout kTagLayout =
                AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

        // The standard deviations of our vision estimated poses, which affect correction rate
        // TODO: (Fake values. Experiment and determine estimation noise on an actual robot.)
        public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
        public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);

    }

  public static final class ModuleConstants {
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
    public static final double kDrivingMotorReduction = (kDrivingWheelBevelGearTeeth * kDrivingWheelFirstStageSpurGearTeeth)
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
    public static final int SHOOTER_FLYWHEEL_CAN_ID = 12; //NEED TO SET
    public static final int FEEDER_CAN_ID = 11;
    public static final double FEEDING_SPEED = 0.75;
    public static final double BACKFEED_SPEED = -0.75;

    //PID constants for nonweighted flywheels
    public static final double kP_nw = 0.00001;
    public static final double kI_nw = 0.0000001;
    public static final double kD_nw = 0.01;

    //PID constants for weighted flywheels
    public static final double kP_w = 0.000005;
    public static final double kI_w = 0.0000001;
    public static final double kD_w = 0.02;

    //from neo vortex vendor website
    public static final double kV_nw = 0.0016;
    public static final double kV_w = 0.0010;

    public static InterpolatingDoubleTreeMap speedMap = new InterpolatingDoubleTreeMap();

    static{
      //              meters      rpm
      speedMap.put(1.5, 2500.0);
      speedMap.put(2.0, 2650.0);
      speedMap.put(2.5, 2850.0);
      speedMap.put(3.0, 3050.0);
      speedMap.put(3.5, 3700.0);
      speedMap.put(4.0, 4000.0);

    }
  }
    public static final class FieldConstants {
    public static final Translation2d blueHubCenterPose = new Translation2d(4.633, 4.035);
    public static final Translation2d redHubCenterPose = new Translation2d(11.907, 4.035);

    public static final Pose2d blueLeftTowerPose = new Pose2d(1.567, 4.180, Rotation2d.fromDegrees(180));
    public static final Pose2d blueRightTowerPose = new Pose2d(1.567, 3.298, Rotation2d.fromDegrees(180));

    public static final Pose2d redLeftTowerPose = new Pose2d(14.984, 3.901, Rotation2d.fromDegrees(0));
    public static final Pose2d redRightTowerPose = new Pose2d(14.984, 4.740, Rotation2d.fromDegrees(0));
  }
}
