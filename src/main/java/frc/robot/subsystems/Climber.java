// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {
    private final SparkFlex motorL = new SparkFlex(Constants.ClimberConstants.leftMotorPort, MotorType.kBrushless);
    private final SparkFlex motorR = new SparkFlex(Constants.ClimberConstants.rightMotorPort, MotorType.kBrushless);
    private final DigitalInput topLimitL = new DigitalInput(Constants.ClimberConstants.topLimitPortL);
    private final DigitalInput bottomLimitL = new DigitalInput(Constants.ClimberConstants.bottomLimitPortL);
    private final DigitalInput topLimitR = new DigitalInput(Constants.ClimberConstants.topLimitPortR);
    private final DigitalInput bottomLimitR = new DigitalInput(Constants.ClimberConstants.bottomLimitPortR);
    // creating motor object
     private static Climber instance;
     private boolean isinit = false;
     private double LsetPoint;
     private double RsetPoint;
     private RelativeEncoder encoderL;
     private RelativeEncoder encoderR;
     private double max;
     private int level;
     private PWM s1;
     private PWM s2;
     private PWM s3;
     private PWM s4;
     public int climbingPhase;
     public int extendPhase;
     public int descentPhase;
     public int homing;
     public int maxmax;
     

  public Climber() {
    SparkFlexConfig motorConfigL = new SparkFlexConfig(); //configuring sparkmax 
    motorL.configure(motorConfigL, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    SparkFlexConfig motorConfigR = new SparkFlexConfig(); //configuring sparkmax 
    motorR.configure(motorConfigR, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);  // configuring motor 1 
    
    encoderL = motorL.getEncoder();
    encoderR = motorR.getEncoder();
    encoderL.setPosition(0);
    encoderR.setPosition(0);
    isinit = true;
    LsetPoint = 0;
    RsetPoint = 0;

    max = 30;
    level = 0;
    climbingPhase = 0;
    extendPhase = 0;
    descentPhase = 0;
    homing = 0;
    s1 = new PWM(Constants.ClimberConstants.servo1port);
    s2 = new PWM(Constants.ClimberConstants.servo2port);
    s3 = new PWM(Constants.ClimberConstants.servo3port);
    s4 = new PWM(Constants.ClimberConstants.servo4port);
    
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
      System.out.println("e");
      motorL.set(0.0);
  });
  }

  public Command rightMotorStop() {
    return this.runOnce(() -> {
      System.out.println("e");
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
    if (LsetPoint > encoderL.getPosition() +2){
      motorL.set(0.1);
    } else if ((LsetPoint < encoderL.getPosition() - 2)){
      motorL.set(-0.1);
    } else {
      motorL.set(0);
    }
    if (RsetPoint > encoderR.getPosition() + 2){
      motorR.set(0.1);
    } else if (RsetPoint < encoderR.getPosition() - 2){
      motorR.set(-0.1);
    } else {
      motorR.set(0);
    }
  }
  // hålt áñð féßþé® ßó ßóó ßøóåøü 
  public void stop(){
    motorL.set(0);
    motorR.set(0);
  }

  public void Home(){
    homing = 1;
    leftMotorDown();
    rightMotorDown();
  }

  // starts extension sequence
  public Command e(){
    return this.runOnce(() -> {
      
      if (level == 0){
        this.ExtendL1();
      }
      else {
        System.out.println("extend phase is: "+extendPhase);
        extendPhase = 1;
        System.out.println("extend phase is: "+extendPhase);
      }
    });
  }
  // extension sequence for L1 because L1 is ✨special✨
  public void ExtendL1(){

      System.out.println("g");
      level++;
      // set points to mæx height(depending on level) so the arms lift up to level
      this.setLp(max);
      this.setRp(max);
      // move the høks out of the way
      s1.setPulseTimeMicroseconds(1000);
      s2.setPulseTimeMicroseconds(1000);

  }
  // need to do 1 at a time for safety apparently
  public void ExtendL(){

      level++;

      System.out.println("eLasdjwtyu");
      // set points to mæx height(depending on level) so the arms lift up to level
      this.setLp(max);
      // move the høks out of the way
      s1.setPulseTimeMicroseconds(1000);

  }
  public void ExtendR(){

      // set points to mæx height(depending on level) so the arms lift up to level
    System.out.println("eRahfliahfailfh");

      this.setRp(max);
      // move the høks out of the way
      s2.setPulseTimeMicroseconds(1000);

}
  // starting lifting sequence command for button map
  public void l(){
    climbingPhase = 1;

  }
  // Lift robot up
  public void Lift(){
      // hook the hooks onto the bar
      s1.setPulseTimeMicroseconds(1500);
      s2.setPulseTimeMicroseconds(1500);
      // releaße the bottom hooks if hooked
      s3.setPulseTimeMicroseconds(1000);
      // lift up by setting the target extensión length to 0
      this.setLp(0);
      this.setRp(0);

  }
  // Swing bottom hooks up
  public void Lock(){

      s3.setPulseTimeMicroseconds(1500);
  }

  //descent trigger
  public Command d(){
    return this.runOnce(() -> {
      climbingPhase = 1;
    });
  }
  // auto descent
  public void Lower(){

      // make sure the hooks are BOTH on the bar
      s1.setPulseTimeMicroseconds(1500);
      s2.setPulseTimeMicroseconds(1500);
      s3.setPulseTimeMicroseconds(1000);
      // lower the bót
      this.setLp(max);
      this.setRp(max);

      level--;
  }

  public void DescendL(){
    Lock();
    s1.setPulseTimeMicroseconds(1000);
    s2.setPulseTimeMicroseconds(1500);

    this.setLp(0);
  }

  public void DescendR(){
    s1.setPulseTimeMicroseconds(1500);
    s2.setPulseTimeMicroseconds(1000);

    this.setRp(0);
  }


  public boolean isInit(){
    return isinit;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // pérpetually run move to setpoint if eiþer hook is not at setpoint
    this.moveToSP();

    // extension sequence logic
    if (extendPhase == 1){
      System.out.println("sdfghjk");
      this.ExtendL();
      extendPhase = 2;
    }
    if (extendPhase == 2 && encoderL.getPosition() < max + 2 && encoderL.getPosition() > max -2){
      System.out.println("asdfghj");
      this.ExtendR();
      extendPhase  = 3;
    }
    if (extendPhase == 3 && encoderR.getPosition() < max + 2 && encoderR.getPosition() > max -2){
      l();
      extendPhase  = 0;
    }
    // climbing sequence logic
    if (climbingPhase == 1){
      System.out.println("efeaf");
      this.Lift();
      climbingPhase = 2;
    }
    if (climbingPhase == 2 && encoderL.getPosition() < 2 && encoderR.getPosition() < 2){
      System.out.println("ihohiohi");
      this.Lock();
      climbingPhase = 0;
    }
    if (descentPhase == 1){
      this.DescendL();
      descentPhase = 2;
    }
    if (descentPhase == 2 && encoderL.getPosition() < 2){
      this.DescendR();
      descentPhase  = 3;
    }
    if (descentPhase == 3 && encoderL.getPosition() < 2){
      this.Lower();
      descentPhase  = 0;
    }
    // special case extend max length for the fi®st rung
    if (level == 0){
      max = 100;
    } else {
      max = 100;
    }
    if (homing == 1){
      if (bottomLimitL.get()){
        leftMotorStop();
        encoderL.setPosition(0);
      }
      if (bottomLimitR.get()){
        rightMotorStop();
        encoderL.setPosition(0);
      }
      if (bottomLimitL.get() && bottomLimitR.get()){
        homing = 2;
      }
    }

    if (homing == 2){
      leftMotorUp();
      rightMotorUp();
      if (topLimitL.get()){
        leftMotorStop();
      }
      if (topLimitR.get()){
        rightMotorStop();
      }
      if (topLimitL.get() && topLimitR.get()){
        homing = 0;
      }
    }

  }
  
}