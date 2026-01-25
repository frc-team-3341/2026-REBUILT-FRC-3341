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
    private final SparkMax motor1 = new SparkMax(20, MotorType.kBrushless); // creating motor object

  public Climber() {
    SparkMaxConfig motorConfig1 = new SparkMaxConfig(); //configuring sparkmax 
    motor1.configure(motorConfig1, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters); // configuring motor 1 
  }

  public Command motorUp() {
    return this.runOnce(() -> {
      motor1.set(0.4);
  });
  }

  public Command motorStop() {
    return this.runOnce(() -> {
      motor1.set(0.0);
  });
  }

  public Command motorDOwn() {
    return this.runOnce(() -> {
      motor1.set(-0.4);
  });
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}