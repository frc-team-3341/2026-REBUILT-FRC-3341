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
  private double velocity = 0;

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

    shooterConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .p(0.0001, ClosedLoopSlot.kSlot0)
        .i(0.0000001, ClosedLoopSlot.kSlot0)
        .d(0, ClosedLoopSlot.kSlot0)
        .outputRange(-1, 1, ClosedLoopSlot.kSlot0)
        .feedForward
          .kV(0.00016, ClosedLoopSlot.kSlot0);// found through trial and error <a href="https://docs.google.com/spreadsheets/d/1mUxeWXwDsTIuaJu80DP9HOvX7ygvbocv1wcLSLS03-A/edit?gid=0#gid=0">  

    shooter.configure(shooterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }



  public void setVelocity(double v) {
    targetRPM = (v * 60.0)/ (Math.PI * ShooterConstants.flywheelDiameter);
    //shooter.setVoltage(2.0);
    closedLoopController.setSetpoint(targetRPM, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
  }
public void setRPM(double rpm) {
  targetRPM = rpm;
  closedLoopController.setSetpoint(rpm,ControlType.kVelocity,ClosedLoopSlot.kSlot0);
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
              (2*(distance*Math.tan(angle)+(initialHeight-ShooterConstants.hubHeight))));
      
      return velocity;
  }
  public Command slowMotor() {
    return this.runOnce(() -> {
        while (velocity != 0) {
          if (velocity > 0) {
            velocity = velocity - 0.2;
          }
          if (velocity < 0) {
            velocity = 0;
          }
          setVelocity(velocity);
      }

    });
  }
  public Command stopMotor() {
    return this.runOnce(() -> {
        velocity = 0;
        setVelocity(velocity);
    });  
  }
  public Command decrementVel() {
    return this.runOnce(() -> {
        velocity -= 0.5;
        setVelocity(velocity);
    });  
  }
  public Command incrementVel(){
    return this.runOnce(() -> {
      velocity += 0.5;
      setVelocity(velocity);
    });
  }


 @Override
  public void periodic() {
  SmartDashboard.putNumber("Target Velocity", velocity);
  SmartDashboard.putNumber("Encoder RPM", relativeEncoder.getVelocity());
  SmartDashboard.putNumber("Target RPM", targetRPM);
    }
  }


