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

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import static frc.robot.Constants.ShooterConstants.speedMap;

import com.ctre.phoenix.motorcontrol.ControlMode;


public class ShooterSubsystem extends SubsystemBase {
  private final int drivingCANId = 1; //NEED TO SET
  private final SparkFlex shooter;
  private double targetRPM = 1000;
  private double velocity = 0;
  private int counter = 0;
  private double Distance;
  private final SparkFlexConfig shooterConfig;
  private SparkClosedLoopController closedLoopController;
  private RelativeEncoder relativeEncoder;
  TalonSRX _motor = new TalonSRX(3);
  

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
    SmartDashboard.putNumber("Distance", 0.0);
  }


 @Override
  public void periodic() {
  Distance = SmartDashboard.getNumber("Distance", 0.0);
  SmartDashboard.putNumber("Encoder RPM", relativeEncoder.getVelocity());
  SmartDashboard.putNumber("Target RPM", targetRPM);
    }

public void setRPM(double rpm) {
  targetRPM = rpm;
  closedLoopController.setSetpoint(rpm,ControlType.kVelocity,ClosedLoopSlot.kSlot0);
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

  public Command stopMotor() {
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
  public Command Score(){
    return this.runOnce(() -> {
      targetRPM = speedMap.get(Distance);
      setRPM(targetRPM);
    });
  }
  public Command runIntake(){
    return this.runOnce(() ->{
       counter++;
       if (counter%2 == 0) {
        _motor.set(ControlMode.PercentOutput, -0.7);
       }
      else {
        _motor.set(ControlMode.PercentOutput, 0.0);
      }
    });
  }

}
