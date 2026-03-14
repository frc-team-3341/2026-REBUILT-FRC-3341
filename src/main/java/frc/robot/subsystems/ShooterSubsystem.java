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
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.ShooterConstants.*;
import frc.robot.Configs;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Superstructure.FeederState;
import frc.robot.subsystems.Superstructure.ShooterState;
import frc.util.ShooterUtil;
public class ShooterSubsystem extends SubsystemBase {

  private final SparkFlex shooter;
  private final SparkFlex feeder;

  private double targetRPM = 0;
  private double distance = 0;
  private double prevDistance = -1;

  private SparkClosedLoopController closedLoopController;

  private RelativeEncoder relativeEncoder;
  private RelativeEncoder feederEncoder;

  private Pose2d robotPose = new Pose2d();
  private DriveSubsystem driveSubsystem;

  public ShooterSubsystem(DriveSubsystem drive) {

    shooter = new SparkFlex(SHOOTER_FLYWHEEL_CAN_ID, MotorType.kBrushless);
    feeder = new SparkFlex(FEEDER_CAN_ID, MotorType.kBrushless);
    
    relativeEncoder = shooter.getEncoder();
    feederEncoder = feeder.getEncoder();

    closedLoopController = shooter.getClosedLoopController();


    shooter.configure(Configs.Shooter.FLYWHEEL_CONFIG, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    feeder.configure(Configs.Shooter.FEEDER_CONFIG, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Init drive subsystem
    driveSubsystem = drive;
    robotPose = driveSubsystem.getOdometryPose();

    
  }


 @Override
  public void periodic() {
    // Update Variables
    distance = getHubDistance();

    SmartDashboard.putNumber("Encoder RPM", relativeEncoder.getVelocity());
    SmartDashboard.putNumber("Target RPM", targetRPM);
    SmartDashboard.putBoolean("Feed", (feederEncoder.getVelocity() > 1.0));
    SmartDashboard.putBoolean("BackFeed", (feederEncoder.getVelocity() < -1.0));
    SmartDashboard.putNumber("Estimated Distance", distance);

  }

  public double getHubDistance() {
    //robotPose = driveSubsystem.getPose();
    // Fallback to odometry pose
    
    robotPose = driveSubsystem.getOdometryPose();
    distance = ShooterUtil.getDistanceToHub(robotPose);
    prevDistance = distance;
    return distance;
    
  }


public void setFlywheelRPM(double RPM) {
  targetRPM = RPM;
  closedLoopController.setSetpoint(targetRPM, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
}

public void stopFlywheel() {
  targetRPM = 0;
  closedLoopController.setSetpoint(targetRPM, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
}

public void feed() {
  feeder.set(FEEDING_SPEED);
}
public void backfeed() {
  feeder.set(BACKFEED_SPEED);
}
public void stopFeed() {
  feeder.set(0);
}

  public Command decrementRPM() {
    return this.runOnce(() -> {
        targetRPM -= 50;
      setFlywheelRPM(targetRPM);
    });  
  }
  public Command incrementRPM(){
    return this.runOnce(() -> {
      targetRPM += 50;
      setFlywheelRPM(targetRPM);
    });
  }
  public Command backupShooting(){
    return this.runOnce(() -> {
      // TODO: change this so the driver can move to a consistent location.
      setFlywheelRPM(3500);
    });
  }

  public Command handleFeederTransition(FeederState desiredState) {
        switch (desiredState) {
            case IDLE:
                return Commands.runOnce(() -> this.stopFeed());

            case FEED:
                return Commands.runOnce(() -> this.feed());

            case BACKFEED:
                return Commands.runOnce(() -> this.backfeed());
                
            default:
                return Commands.print("Invalid Feeder State Provided!");

        }
    }

    public Command handleShooterTransitions(ShooterState desiredState) {
        switch (desiredState) {
            case IDLE:
                return this.runOnce(() -> this.stopFlywheel());

            case PASSING:
                return this.runOnce(() -> this.setFlywheelRPM(PASSING_RPM));

            case SCORING:
                return this.run(() -> {
                
                  double distanceToHub = ShooterUtil.getDistanceToHub(robotPose);

                  if (Math.abs(distanceToHub-prevDistance)>0.05) {
                    System.out.println("Updating speed for the distance: " + distanceToHub + " meters from the center of the hub");
                    targetRPM = speedMap.get(distanceToHub);
                    prevDistance = distanceToHub;
                  }

                  this.setFlywheelRPM(targetRPM);
                  //before starting get distance to hub so not storing 0.0 in lastPose
                  }).beforeStarting(() -> {
                    prevDistance = ShooterUtil.getDistanceToHub(robotPose);
                    //get flywheel spinning to prevent delay
                    targetRPM = speedMap.get(prevDistance);
                  });
                    
            default:
                return Commands.print("Invalid Shooter State Provided!");
        }
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
