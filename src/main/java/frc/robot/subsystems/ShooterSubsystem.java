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

import static frc.robot.Constants.ShooterConstants.*;

public class ShooterSubsystem extends SubsystemBase {

  private final SparkFlex shooter;
  private final SparkFlex feeder;
  private double targetRPM = 0;
  private double velocity = 0;
  private int counter = 0;
  private double Distance;
  private final SparkFlexConfig shooterConfig;
  private final SparkFlexConfig feederConfig;
  private SparkClosedLoopController closedLoopController;
  private SparkClosedLoopController closedLoopFeedController;
  private RelativeEncoder relativeEncoder;
  private RelativeEncoder feederEncoder;

  public ShooterSubsystem() {
    shooter = new SparkFlex(SHOOTER_FLYWHEEL_CAN_ID, MotorType.kBrushless);
    feeder = new SparkFlex(FEEDER_CAN_ID, MotorType.kBrushless);
    
    relativeEncoder = shooter.getEncoder();
    feederEncoder = feeder.getEncoder();

    shooterConfig = new SparkFlexConfig();
    feederConfig = new SparkFlexConfig();

    feederConfig
        .smartCurrentLimit(80)
        .idleMode(IdleMode.kBrake);

    shooterConfig
        .smartCurrentLimit(80)
        .idleMode(IdleMode.kCoast);

    shooterConfig.
      encoder
        .inverted(false);
    feederConfig.
      encoder
        .inverted(false);

    closedLoopController = shooter.getClosedLoopController();
    closedLoopFeedController = feeder.getClosedLoopController();

    shooterConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .p(kP_nw, ClosedLoopSlot.kSlot0)
        .i(kI_nw, ClosedLoopSlot.kSlot0)
        .d(kD_nw, ClosedLoopSlot.kSlot0)
        .outputRange(-1, 1, ClosedLoopSlot.kSlot0)
        .feedForward
          .kV(kV, ClosedLoopSlot.kSlot0);


    shooter.configure(shooterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    feeder.configure(feederConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    SmartDashboard.putNumber("Distance", 0.0);
  }


 @Override
  public void periodic() {
  Distance = SmartDashboard.getNumber("Distance", 0.0);
  SmartDashboard.putNumber("Encoder RPM", relativeEncoder.getVelocity());
  SmartDashboard.putNumber("Target RPM", targetRPM);
  SmartDashboard.putBoolean("Feed", (feederEncoder.getVelocity() > 1.0));
  SmartDashboard.putBoolean("BackFeed", (feederEncoder.getVelocity() < -1.0));
    }

public void setRPM(double rpm) {
  targetRPM = rpm;
  closedLoopController.setSetpoint(rpm,ControlType.kVelocity,ClosedLoopSlot.kSlot0);
}

public void startFeedMotor() {
  feeder.set(FEEDING_SPEED);
}
public void startBackFeed() {
  feeder.set(BACKFEED_SPEED);
}
public void stopFeedMotor() {
  feeder.set(0);
}

  public Command stopFlywheel() {
    return this.runOnce(() -> {
        velocity = 0;
        setRPM(velocity);
    });  
  }
  public Command decrementRPM() {
    return this.runOnce(() -> {
        targetRPM -= 100;
      setRPM(targetRPM);
    });  
  }
  public Command incrementRPM(){
    return this.runOnce(() -> {
      targetRPM += 100;
      setRPM(targetRPM);
    });
  }
  public Command backupShooting(){
    return this.runOnce(() -> {
      setRPM(3500);
    });
  }
  public Command Score(){
    return this.runOnce(() -> {
      targetRPM = speedMap.get(Distance);
      setRPM(targetRPM);
    });
  }
  public Command feed(){
    return this.runOnce(() ->{
       startFeedMotor();
    });
  }

  public Command stopFeed() {
    return this.runOnce(() -> {
      stopFeedMotor();
    });
  }

/* 
  public double calculateLinearLaunchVelocity(double distance, double initialHeight, double center_offset) {
      double angle = Math.toRadians(75);

      double velocity = ((distance+center_offset)/(Math.cos(angle))) * 
          Math.sqrt(9.81/
              (2*((distance+center_offset)*Math.tan(angle)+(initialHeight-ShooterConstants.hubHeight))));
      
      return velocity;
  }
*/
}
