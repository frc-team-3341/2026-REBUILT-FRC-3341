package frc.robot;

import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.ShooterConstants;

import static frc.robot.Constants.DriveConstants.*;
import static frc.robot.Constants.ShooterConstants.*;
import static frc.robot.Constants.IntakeConstants.*;

public final class Configs {
  public static final class EasySwerveModule {
    public static final SparkFlexConfig drivingConfig = new SparkFlexConfig();
    public static final SparkFlexConfig turningConfig = new SparkFlexConfig();


    static {
      // Use module constants to calculate conversion factors and feed forward gain.
      double drivingFactor = ModuleConstants.kWheelDiameterMeters * Math.PI 
        / ModuleConstants.kDrivingMotorReduction;
      double turningFactor = 2 * Math.PI;

      double nominalVoltage = 12.0;
      double drivingVelocityFeedForward = nominalVoltage / ModuleConstants.kDriveWheelFreeSpeedRps;

      drivingConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(drivingCurrentLimit);   // 70 for SparkFlex
      drivingConfig
        .encoder
          .positionConversionFactor(drivingFactor) // meters
          .velocityConversionFactor(drivingFactor / 60.0); // meters per second
      drivingConfig
        .closedLoop
          .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
          // These are example gains you may need to adjust them for your own robot!
          .pid(drivingPID.kP, drivingPID.kI, drivingPID.kD)
          .outputRange(-1, 1)
        .feedForward
          .kV(drivingVelocityFeedForward);

      turningConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(turningCurrentLimit);   // 70 for SparkFlex
      turningConfig
        .absoluteEncoder
          // Do not invert the turning encoder, since the output shaft rotates in the same
          // direction as the steering motor in the EasySwerve Module.
          .inverted(false)
          .positionConversionFactor(turningFactor) // radians
          .velocityConversionFactor(turningFactor / 60.0) // radians per second
          // This applies to REV Through Bore Encoder V2 (use REV_ThroughBoreEncoder for V1):
          .apply(AbsoluteEncoderConfig.Presets.REV_ThroughBoreEncoderV2);

      turningConfig
        .closedLoop
          .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
          // These are example gains you may need to adjust them for your own robot!
          .pid(turningPID.kP, turningPID.kI, turningPID.kD)
          .outputRange(-1, 1)
          // Enable PID wrap around for the turning motor. This will allow the PID
          // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
          // to 10 degrees will go through 0 rather than the other direction which is a
          // longer route.
          .positionWrappingEnabled(true)
          .positionWrappingInputRange(0, turningFactor);
    }
  }

  public static final class Shooter {
    public static final SparkFlexConfig FLYWHEEL_CONFIG = new SparkFlexConfig();

    //TODO needs to be configured
    public static final SparkFlexConfig FEEDER_CONFIG = new SparkFlexConfig();

    static {
      FLYWHEEL_CONFIG
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(shooterCurrentLimit);
      
      FLYWHEEL_CONFIG
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(kP, kI, kD)
        .outputRange(-1, 1)
        .feedForward
          .kV(kV);

      FEEDER_CONFIG
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(shooterCurrentLimit);

    }
  }

  public static final class Intake {
    public static final SparkFlexConfig INTAKE_CONFIG = new SparkFlexConfig();

    static {
      INTAKE_CONFIG
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(intakeCurrentLimit);
    }
  }
}