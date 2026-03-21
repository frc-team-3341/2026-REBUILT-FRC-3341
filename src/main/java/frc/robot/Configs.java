package frc.robot;

import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import frc.robot.Constants.ModuleConstants;
import static frc.robot.Constants.IntakeConstants.*;
import static frc.robot.Constants.ShooterConstants.*;

public final class Configs {
  public static final class EasySwerveModule {
    public static final SparkMaxConfig drivingConfig = new SparkMaxConfig();
    public static final SparkMaxConfig turningConfig = new SparkMaxConfig();

    static {
      // Use module constants to calculate conversion factors and feed forward gain.
      double drivingFactor = ModuleConstants.kWheelDiameterMeters * Math.PI 
        / ModuleConstants.kDrivingMotorReduction;
      double turningFactor = 2 * Math.PI;

      double nominalVoltage = 12.0;
      double drivingVelocityFeedForward = nominalVoltage / ModuleConstants.kDriveWheelFreeSpeedRps;

      drivingConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(40);   // 70 for SparkFlex
      drivingConfig
        .encoder
          .positionConversionFactor(drivingFactor) // meters
          .velocityConversionFactor(drivingFactor / 60.0); // meters per second
      drivingConfig
        .closedLoop
          .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
          // These are example gains you may need to adjust them for your own robot!
          .pid(0.04, 0, 0)
          .outputRange(-1, 1)
        .feedForward
          .kV(drivingVelocityFeedForward);

      turningConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(40);   // 70 for SparkFlex
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
          .pid(1, 0, 0)
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
        .pid(kP_w, kI_w, kD_w)
        .outputRange(-1, 1)
        .feedForward
          .kV(kV_w);

      FEEDER_CONFIG
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(shooterCurrentLimit);

    }
  }

  public static final class Intake {
    public static final SparkFlexConfig INTAKE_CONFIG = new SparkFlexConfig();
    public static final SparkFlexConfig LIFT_CONFIG = new SparkFlexConfig();

    static {
      INTAKE_CONFIG
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(intakeCurrentLimit);
        
      // INTAKE_CONFIG
      //   .closedLoop
      //   .feedbackSensor(FeedbackSensor.kPrimaryEncoder);
      
      // INTAKE_CONFIG
      //   .encoder
      //   .inverted(true);
      
      LIFT_CONFIG
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(intakeCurrentLimit);
    }
  }
}
