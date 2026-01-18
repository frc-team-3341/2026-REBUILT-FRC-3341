// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
  private final int drivingCANId = 0; //NEED TO SET
  private final SparkFlex shooter;
  private final SparkFlexConfig shooterConfig;
  private SparkClosedLoopController closedLoopController;
  private RelativeEncoder shooterEncoder;
  
  /** Creates a new Shooter. */
  public ShooterSubsystem() {
    shooter = new SparkFlex(drivingCANId, MotorType.kBrushless);
    shooterConfig = new SparkFlexConfig();
    shooterConfig
        .smartCurrentLimit(80)
        .idleMode(IdleMode.kCoast);

    // Persist parameters to retain configuration in the event of a power cycle
    shooter.configure(shooterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    closedLoopController = shooter.getClosedLoopController();
    shooterEncoder = shooter.getEncoder();

    /*
     * Configure the closed loop controller. We want to make sure we set the
     * feedback sensor as the primary encoder.
     */
    shooterConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        // Set PID values for position control. We don't need to pass a closed loop
        // slot, as it will default to slot 0.
        // .p(0.1)
        // .i(0)
        // .d(0)
        // .outputRange(-1, 1)

        // Set PID values for velocity control in slot 1
        .p(0.0001, ClosedLoopSlot.kSlot1)
        .i(0, ClosedLoopSlot.kSlot1)
        .d(0, ClosedLoopSlot.kSlot1)
        .outputRange(-1, 1, ClosedLoopSlot.kSlot1)
        .feedForward
          // kV is now in Volts, so we multiply by the nominal voltage (12V)
          .kV(12.0 / 5767, ClosedLoopSlot.kSlot1);
    
    SmartDashboard.setDefaultNumber("Target Velocity", 0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    double targetVelocity = SmartDashboard.getNumber("Target Velocity (RPM)", 0);
    closedLoopController.setSetpoint(targetVelocity, ControlType.kVelocity, ClosedLoopSlot.kSlot1);
  }
}
