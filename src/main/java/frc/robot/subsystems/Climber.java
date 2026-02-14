// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

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
    private final SparkMax motorL = new SparkMax(2, MotorType.kBrushless);
    private final SparkMax motorR = new SparkMax(3, MotorType.kBrushless);
     // creating motor object
     private static Climber instance;
     private boolean isinit = false;
     private double LsetPoint;
     private double RsetPoint;
     private RelativeEncoder encoderL;
     private RelativeEncoder encoderR;
     private double max;
     private int level;
     private Servo s1;
     private Servo s2;
     private Servo s3;
     private Servo s4;
     public int climbingPhase;
     public int extendPhase;
     

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
    climbingPhase = 0;
    extendPhase = 0;
    s1 = new Servo(0);
    s2 = new Servo(1);
    s3 = new Servo(2);
    s4 = new Servo(3);
    
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
  // sets the LEFT seT point
  public void setLp(double s){
    LsetPoint = s;
  }
  // sets the rIGHT seT point
  public void setRp(double s){
    RsetPoint = s;
  }
  // move to set point
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
  // hålt áñð féßþé® ßó ßóó ßøóåøü 
  public Command stop(){
    return this.runOnce(() -> {
      leftMotorStop();
      rightMotorStop();
  });
  }
  // starts extension sequence
  public Command e(){
    return this.runOnce(() -> {
      
      if (level == 0){
        ExtendL1();
      }
      else {
        extendPhase = 1;
      }
    });
  }
  // extension sequence for L1 because L1 is ✨special✨
  public Command ExtendL1(){
    return this.runOnce(() -> {
      System.out.println("g");
      level++;
      // set points to mæx height(depending on level) so the arms lift up to level
      setLp(max);
      setRp(max);
      // move the høks out of the way
      s1.set(1000);
      s2.set(1000);

  });
  }
  // need to do 1 at a time for safety apparently
  public Command ExtendL(){
    return this.runOnce(() -> {
      level++;
      // set points to mæx height(depending on level) so the arms lift up to level
      setLp(max);
      // move the høks out of the way
      s1.set(1000);

  });
  }
  public Command ExtendR(){
    return this.runOnce(() -> {
      // set points to mæx height(depending on level) so the arms lift up to level
      setRp(max);
      // move the høks out of the way
      s2.set(1000);

  });}
  // starting lifting sequence command for button map
  public Command l(){
    return this.runOnce(() -> {
      climbingPhase = 1;
    });
  }
  // Lift robot up
  public Command Lift(){
    return this.runOnce(() -> {
      // hook the hooks onto the bar
      s1.set(1500);
      s2.set(1500);
      // releaße the bottom hooks if hooked
      s3.set(1000);
      s4.set(1000);
      // lift up by setting the target extensión length to 0
      setLp(0);
      setRp(0);
  });
  }
  // Swing bottom hooks up
  public Command Lock(){
    return this.runOnce(() -> {
      s3.set(1500);
      s4.set(1500);
    });
  }
  // auto descent
  public Command Descend(){
    return this.runOnce(() -> {
      // make sure the hooks are BOTH on the bar
      s1.set(1500);
      s2.set(1500);
      // lower the bót
      setLp(max);
      setRp(max);
    });
  }

  public boolean isInit(){
    return isinit;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // pérpetually run move to setpoint if eiþer hook is not at setpoint
    if(LsetPoint != encoderL.getPosition() || RsetPoint != encoderR.getPosition())
      moveToSP();
    // extension sequence logic
    if (extendPhase == 1){
      ExtendL();
      extendPhase = 2;
    }
    if (extendPhase == 2 && encoderL.getPosition() > max - 0.05 && encoderL.getPosition() < max +0.05){
      ExtendR();
      extendPhase  = 0;
    }
    // climbing sequence logic
    if (climbingPhase == 1){
      Lift();
      climbingPhase = 2;
    }
    if (climbingPhase == 2 && encoderL.getPosition() < 0.05 && encoderR.getPosition() < 0.05){
      Lock();
      climbingPhase = 0;
    }
    // special case extend max length for the fi®st rung
    if (level == 0){
      max = 100;
    } else {
      max = 80;
    }
  }
  
}