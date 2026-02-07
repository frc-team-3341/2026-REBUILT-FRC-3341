// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
    private final SparkMax motorL = new SparkMax(20, MotorType.kBrushless);
    private final SparkMax motorR = new SparkMax(20, MotorType.kBrushless);
     // creating motor object
     private final Servo hookL = new Servo(0);
     private final Servo hookR = new Servo(0);
     private static Climber instance;
     private boolean isinit = false;
     private double LsetPoint;
     private double RsetPoint;
     private RelativeEncoder encoderL;
     private RelativeEncoder encoderR;
     private double max;
     private int level;
     

  public Climber() {
    SparkMaxConfig motorConfigL = new SparkMaxConfig(); //configuring sparkmax 
    motorL.configure(motorConfigL, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    SparkMaxConfig motorConfigR = new SparkMaxConfig(); //configuring sparkmax 
    motorR.configure(motorConfigR, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);  // configuring motor 1 
    encoderL = motorL.getEncoder();
    encoderR = motorR.getEncoder();
    isinit = true;
    LsetPoint = 0;
    RsetPoint = 0;
    max = 30;
    level = 0;
  }

  public static Climber getInstance(){
    if(instance == null){
      instance = new Climber();
    }
    return instance;
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

  public void setLp(double s){
    LsetPoint = s;
  }
  public void setRp(double s){
    RsetPoint = s;
  }

  public void moveToSP(){
    if (LsetPoint > encoderL.getPosition()){
      leftMotorUp();
    } else if ((LsetPoint < encoderL.getPosition())){
      leftMotorDown();
    }
    if (RsetPoint > encoderR.getPosition()){
      rightMotorUp();
    } else if (RsetPoint < encoderR.getPosition()){
      rightMotorDown();
    }
  }

  public Command stop(){
    return this.runOnce(() -> {
      leftMotorStop();
      rightMotorStop();
  });
  }
  public Command Ascend(){
    return this.runOnce(() -> {
      level++;
      if (level < 4){
        // values are temporary - need to be tested
        if (level == 1){
          max = 30;
        }
        else {
          max = 20;
          if(encoderR.getPosition() < 0.05){
            setRp(max);
            hookR.set(0);
          } else {
            setRp(0);
            hookR.set(0.5);
          }
        }
        if(encoderL.getPosition() < 0.05){
          setLp(max);
          hookL.set(0);
        } else {
          setLp(0);
          hookL.set(0.5);
        }
      } else {
        if (encoderL.getPosition() > max - 0.05){
          hookL.set(0.5);
        } else {
          hookL.set(0);
        }
        if (encoderR.getPosition() > max - 0.05){
          hookR.set(0.5);
        } else {
          hookR.set(0);
        }
        setLp(0);
        setRp(0);
      }

      
      
  });
  }
  public Command Descend(){
    return this.runOnce(() -> {
      level--;
      if (level > 0){
        if (level == 1){
          max = 30;
        }
        else {
          max = 20;
        }
        if(encoderL.getPosition() < 0.05){
          setLp(max);
          hookL.set(0.5);
        } else {
          setLp(0);
          hookL.set(0);
        }
        if(encoderR.getPosition() < 0.05){
          setRp(max);
          hookR.set(0.5);
          } else {
            setRp(0);
            hookR.set(0);
          }
      } else {
        if (encoderL.getPosition() > max - 0.05){
          hookL.set(0.5);
        }
        if (encoderR.getPosition() > max - 0.05){
          hookR.set(0.5);
        }
        setLp(max);
        setRp(max);
      }

  });
  }

  public boolean isInit(){
    return isinit;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if(LsetPoint != encoderL.getPosition() || RsetPoint != encoderR.getPosition())
      moveToSP();
  }
}