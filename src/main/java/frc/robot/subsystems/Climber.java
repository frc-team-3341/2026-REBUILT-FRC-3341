// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
    private final SparkMax motorL = new SparkMax(20, MotorType.kBrushless);
    private final SparkMax motorR = new SparkMax(20, MotorType.kBrushless); // creating motor object

  public Climber() {
    SparkMaxConfig motorConfigL = new SparkMaxConfig(); //configuring sparkmax 
    motorL.configure(motorConfigL, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    SparkMaxConfig motorConfigR = new SparkMaxConfig(); //configuring sparkmax 
    motorR.configure(motorConfigR, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);  // configuring motor 1 
  }

  public Command leftMotorUp() {
    return this.runOnce(() -> {
      motorL.set(0.4);
  });

  }

  public Command rightMotorUp() {
    return this.runOnce(() -> {
      motorR.set(0.4);
  });

  }

  public Command leftMotorStop() {
    return this.runOnce(() -> {
      motorL.set(0.0);
  });
  }

  public Command rightMotorStop() {
    return this.runOnce(() -> {
      motorR.set(0.0);
  });
  }

  public Command leftMotorDown() {
    return this.runOnce(() -> {
      motorL.set(-0.4);
  });
  }
  public Command rightMotorDown() {
    return this.runOnce(() -> {
      motorR.set(-0.4);
  });
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}