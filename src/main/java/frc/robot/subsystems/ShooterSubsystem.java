// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
 
package frc.robot.subsystems;
import java.lang.Math;
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

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.ShooterConstants.*;

import frc.util.ShooterUtil;
public class ShooterSubsystem extends SubsystemBase {

  private final SparkFlex shooter;
  private final SparkFlex feeder;
  private double targetRPM = 0;
  private double velocity = 0;
  private double distance = 0;
  private double prevdistance = -1;
  private final SparkFlexConfig shooterConfig;
  private final SparkFlexConfig feederConfig;
  private SparkClosedLoopController closedLoopController;
  private RelativeEncoder relativeEncoder;
  private RelativeEncoder feederEncoder;
  private Pose2d robotPose = new Pose2d();
  private DriveSubsystem driveSubsystem;
  private double kP = kP_w;
  private double kI = kI_w;
  private double kD = kD_w;
  private double kV = kV_w;

  public ShooterSubsystem(DriveSubsystem drive) {

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

    closedLoopController = shooter.getClosedLoopController();

    shooterConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .p(kP_w, ClosedLoopSlot.kSlot0)
        .i(kI_w, ClosedLoopSlot.kSlot0)
        .d(kD_w, ClosedLoopSlot.kSlot0)
        .outputRange(-1, 1, ClosedLoopSlot.kSlot0)
        .feedForward
          .kV(kV_w, ClosedLoopSlot.kSlot0);


    shooter.configure(shooterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    feeder.configure(feederConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Init drive subsystem
    driveSubsystem = drive;
    robotPose = driveSubsystem.getOdometryPose();

    
  }


 @Override
  public void periodic() {
    // Update Variables
    distance = getHubDistance();

    SmartDashboard.putNumber("kP", kP);
    SmartDashboard.putNumber("kI", kI);
    SmartDashboard.putNumber("kD", kD);
    SmartDashboard.putNumber("kV", kV);

    kP = SmartDashboard.getNumber("kP", kP);
    kI = SmartDashboard.getNumber("kI", kI);
    kD = SmartDashboard.getNumber("kD", kD);
    kV = SmartDashboard.getNumber("kV", kV);

    SmartDashboard.putNumber("Encoder RPM", relativeEncoder.getVelocity());
    SmartDashboard.putNumber("Target RPM", targetRPM);
    SmartDashboard.putBoolean("Feed", (feederEncoder.getVelocity() > 1.0));
    SmartDashboard.putBoolean("BackFeed", (feederEncoder.getVelocity() < -1.0));
    SmartDashboard.putNumber("Estimated Distance", distance);

    setP(kP);
    setI(kI);
    setD(kD);
    setkV(kV);

  }

  public double getHubDistance() {
    robotPose = driveSubsystem.getPose();
    // Fallback to odometry pose
    // robotPose = driveSubsystem.getOdometryPose();
    
    // robotPose = driveSubsystem.getOdometryPose();
    distance = ShooterUtil.getDistanceToHub(robotPose);
    prevdistance = distance;
    return distance;
    
  }


public void setRPM(double rpm) {
  targetRPM = rpm;
  closedLoopController.setSetpoint(rpm,ControlType.kVelocity,ClosedLoopSlot.kSlot0);
}

public void setP(double newP) {
  shooterConfig.closedLoop.p(newP);
}
public void setI(double newI) {
  shooterConfig.closedLoop.i(newI);
}
public void setD(double newD) {
  shooterConfig.closedLoop.d(newD);
}
public void setkV(double newkV) {
  shooterConfig.closedLoop.feedForward.kV(newkV);
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
        targetRPM -= 50;
      setRPM(targetRPM);
    });  
  }
  public Command incrementRPM(){
    return this.runOnce(() -> {
      targetRPM += 50;
      setRPM(targetRPM);
    });
  }
  public Command backupShooting(){
    return this.runOnce(() -> {
      // TODO: change this so the driver can move to a consistent location.
      setRPM(3050);
    });
  }
  public Command score(){
    return this.runOnce(() -> {
      targetRPM = speedMap.get(getHubDistance());
      setRPM(targetRPM);
    });
  }

  public Command feed(){
    return this.runOnce(() ->{
       startFeedMotor();
    });
  }

  public Command backfeed(){
    return this.runOnce(() -> {
      startBackFeed();
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
