// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
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
    public static final double kFrontLeftChassisAngularOffset = (-Math.PI / 2) + kEasySwerveAngularOffsetCompensation;
    public static final double kFrontRightChassisAngularOffset = 0 + kEasySwerveAngularOffsetCompensation;
    public static final double kRearLeftChassisAngularOffset = Math.PI + kEasySwerveAngularOffsetCompensation;
    public static final double kRearRightChassisAngularOffset = (Math.PI / 2) + kEasySwerveAngularOffsetCompensation;

    // The EasySwerve module allows installation of the motors either on top or bottom of the module.
    // These constants configure the location of the motors. The default configuration is with both
    // motors on the bottom of the module.
    public static final boolean kFrontLeftDrivingMotorOnBottom = true;
    public static final boolean kRearLeftDrivingMotorOnBottom = true;
    public static final boolean kFrontRightDrivingMotorOnBottom = true;
    public static final boolean kRearRightDrivingMotorOnBottom = true;

    public static final boolean kFrontLeftTurningMotorOnBottom = true;
    public static final boolean kRearLeftTurningMotorOnBottom = true;
    public static final boolean kFrontRightTurningMotorOnBottom = true;
    public static final boolean kRearRightTurningMotorOnBottom = true;

    // SPARK MAX CAN IDs
    //TODO: WE NEED TO CHANGE THESE BEFORE TESTING!!!!!
    public static final int kFrontLeftDrivingCanId = 0;
    public static final int kRearLeftDrivingCanId = 0;
    public static final int kFrontRightDrivingCanId = 0;
    public static final int kRearRightDrivingCanId = 0;

    public static final int kFrontLeftTurningCanId = 0;
    public static final int kRearLeftTurningCanId = 0;
    public static final int kFrontRightTurningCanId = 0;
    public static final int kRearRightTurningCanId = 0;

    public static final boolean kGyroReversed = false;
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
}