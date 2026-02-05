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

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
  private final int drivingCANId = 1; //NEED TO SET
  private final SparkFlex shooter;
  private double targetRPM = 0;
  private int velocity = 0;

  private final SparkFlexConfig shooterConfig;
  private SparkClosedLoopController closedLoopController;
  private RelativeEncoder relativeEncoder;


  public ShooterSubsystem() {
    shooter = new SparkFlex(drivingCANId, MotorType.kBrushless);
    relativeEncoder = shooter.getEncoder();
    shooterConfig = new SparkFlexConfig();
    shooterConfig
        .smartCurrentLimit(80)
        .idleMode(IdleMode.kCoast);

    closedLoopController = shooter.getClosedLoopController();
    // shooterConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);

    shooterConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .p(0.0001, ClosedLoopSlot.kSlot0)
        .i(0, ClosedLoopSlot.kSlot0)
        .d(0, ClosedLoopSlot.kSlot0)
        .outputRange(-1, 1, ClosedLoopSlot.kSlot0)
        .feedForward
          .kV(12.0 / 5767, ClosedLoopSlot.kSlot0);

    shooter.configure(shooterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    }


  // public void setVelocity(double v) {
  //   targetRPM =(v * 60.0) / (Math.PI * diameter);
  // }

  public void setVelocity(double v) {
    targetRPM = (v * 60.0) * 39.37/ (Math.PI * ShooterConstants.flywheelDiameter);
  }
  
/**
 * Uses conservation of angular momentum to calculate the required
 * RPM to launch the ball out at the desired velocity
 * 
 * Might need to update this method to account for loss of energy
 * 
 * @param velocity
 * 
 * Represents the desired launch velocity in m/s
 * 
 * @see <a href="https://drive.google.com/file/d/1pNwEd03rzBCn8M5KGe_-abfVvG_O993O/view?usp=sharing">Derivation for the formula used</a>

 */

  public void calculateLaunchRPM(double velocity) {

    double flywheelRadius = ShooterConstants.flywheelDiameter/2;

    double finalAngularVelocity = (velocity/flywheelRadius);

    double initialAngularVelocity = finalAngularVelocity + 
      (ShooterConstants.flywheelMass*velocity*flywheelRadius)/(ShooterConstants.flywheelMomentofInertia);
    
    targetRPM = (initialAngularVelocity*60)/(2*Math.PI); 
  } 

  /**
   * Uses basic projectile motion to calculate the linear launch velocity necessary to shoot a
   * projectile a specified distance (assuming a 75° launch angle). Neglects air resistance
   * 
   * 
   * @param distance
   * The desired distance in meters (this represents how far  away the projectile will land 
   * relative to the launch position)
   * 
   * @param initialHeight
   * The initial height of the projectile in meters
   * 
   * @return
   * The required launch velocity in meters/second
   * 
   * @see <a href="https://drive.google.com/file/d/1QK_I150SMKgldrvAcl2610z9DPRuNhpH/view?usp=sharing">Derivation for the formula used</a>
   */
  public double calculateLinearLaunchVelocity(double distance, double initialHeight) {
      double angle = Math.toRadians(75);

      double velocity = ((distance)/(Math.cos(angle))) * 
          Math.sqrt(9.81/
              (2*(distance*Math.tan(angle)+initialHeight)));
      
      return velocity;
  }

  public Command stopMotor() {
    return this.runOnce(() -> {
        velocity = 0;
        setVelocity(velocity);
    });  
  }

  public Command incrementVel(){
    return this.runOnce(() -> {
      velocity++;
      setVelocity(velocity);
    });
  }


 @Override
  public void periodic() {
    closedLoopController.setSetpoint(targetRPM, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
  }
}