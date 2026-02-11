// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import java.util.Optional;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.FieldConstants;
import frc.util.JoystickUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {
  // Create EasySwerveModules
  private final EasySwerveModule m_frontLeft = new EasySwerveModule(
      DriveConstants.kFrontLeftDrivingCanId,
      DriveConstants.kFrontLeftTurningCanId,
      DriveConstants.kFrontLeftChassisAngularOffset,
      DriveConstants.kFrontLeftDrivingMotorOnBottom,
      DriveConstants.kFrontLeftTurningMotorOnBottom);

  private final EasySwerveModule m_frontRight = new EasySwerveModule(
      DriveConstants.kFrontRightDrivingCanId,
      DriveConstants.kFrontRightTurningCanId,
      DriveConstants.kFrontRightChassisAngularOffset,
      DriveConstants.kFrontRightDrivingMotorOnBottom,
      DriveConstants.kFrontRightTurningMotorOnBottom);

  private final EasySwerveModule m_rearLeft = new EasySwerveModule(
      DriveConstants.kRearLeftDrivingCanId,
      DriveConstants.kRearLeftTurningCanId,
      DriveConstants.kRearLeftChassisAngularOffset,
      DriveConstants.kRearLeftDrivingMotorOnBottom,
      DriveConstants.kRearLeftTurningMotorOnBottom);

  private final EasySwerveModule m_rearRight = new EasySwerveModule(
      DriveConstants.kRearRightDrivingCanId,
      DriveConstants.kRearRightTurningCanId,
      DriveConstants.kRearRightChassisAngularOffset,
      DriveConstants.kRearRightDrivingMotorOnBottom,
      DriveConstants.kRearRightTurningMotorOnBottom);

  private EasySwerveModule[] modules;
  private final StructArrayPublisher<SwerveModuleState> statePublisher;
  private final StructPublisher<Pose2d> poseEstimatorPublisher;

  // The gyro sensor
  private AHRS navx = new AHRS(NavXComType.kMXP_SPI);

  //pls tune this controller is kinda wacky lol
  private PIDController arcDriveController = new PIDController(0.2, 0, 0);

  // Odometry class for tracking robot pose
  SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(
      DriveConstants.kDriveKinematics,
      Rotation2d.fromDegrees(navx.getYaw()),
      new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
      });

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    // Usage reporting for EasySwerve template
    // HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_MaxSwerve);
    //TODO (dave): add EasySwerve to tInstances??? 

    modules = new EasySwerveModule[4];

    modules[0] = m_frontLeft;
    modules[1] = m_frontRight;
    modules[2] = m_rearLeft;
    modules[3] = m_rearRight;
    
    statePublisher = NetworkTableInstance.getDefault().getStructArrayTopic("/SwerveStates", SwerveModuleState.struct).publish();
    poseEstimatorPublisher = NetworkTableInstance.getDefault().getStructTopic("/EstimatedPose", Pose2d.struct).publish();

  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    m_odometry.update(
        Rotation2d.fromDegrees(getYaw()),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        });
    
    poseEstimatorPublisher.set(getPose());
    
    SmartDashboard.putNumber("navx angle", navx.getAngle());

    
      

    statePublisher.set(getStates());
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(
        Rotation2d.fromDegrees(getYaw()),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        },
        pose);
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    // Convert the commanded speeds into the correct units for the drivetrain

    ChassisSpeeds robotSpeeds = fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot,
                navx.getRotation2d())
            : new ChassisSpeeds(xSpeed, ySpeed, rot);

    //Test with discretization asap
    // robotSpeeds = ChassisSpeeds.discretize(robotSpeeds, 0.02);

    SwerveModuleState[] swerveModuleStates = 
      DriveConstants.kDriveKinematics.toSwerveModuleStates(robotSpeeds);
    
    setModuleStates(swerveModuleStates);
  }

  //WORKS (only for blue alliance for now)
  public void aimDrive(double xSpeed, double ySpeed) { 

      Pose2d currentPose = getPose();

      Pose2d hubCenterPose = new Pose2d();

      Optional<Alliance> alliance = DriverStation.getAlliance(); 

      int offset = 0;

      //end method if there is no alliance selected cuz this should only be used on the field
      if(!alliance.isPresent()) {
        return;
      }

      if (alliance.get() == Alliance.Blue) {
        hubCenterPose = FieldConstants.blueHubCenterPose;
        offset = 0;
      }
      
      else {
          hubCenterPose = FieldConstants.redHubCenterPose;
          offset = 180;
      } 
      
      double theta;

      double xDisplacement = (hubCenterPose.getX() - currentPose.getX()); 
      double yDisplacement = (hubCenterPose.getY() - currentPose.getY());

      if (xDisplacement != 0) { 
        theta = offset+Math.toDegrees(Math.atan(yDisplacement/xDisplacement)); 
      } 

      //this is incorrect currently cuz there are two places where x is 0, which means the angle
      //can either be 90 or 270
      else {
        theta = offset+90;
      }

      SmartDashboard.putNumber("theta offset", theta);
      SmartDashboard.putNumber("x displacement", xDisplacement);
      SmartDashboard.putNumber("y displacement", yDisplacement);
      
      double rotOutput = arcDriveController.calculate(getYaw(), theta); 

      drive(xSpeed, ySpeed, rotOutput, true);
  }



  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    
    for (int i = 0; i < modules.length; i++) {
      modules[i].setDesiredState(desiredStates[i]);
    }
  }

  public Rotation2d getRotation2d() {
    return navx.getRotation2d();
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    for (int i = 0; i < modules.length; i++) {
      modules[i].resetEncoders();
    }
  }

  /** Zeroes the heading of the robot. */
  public Command zeroHeading() {
    return this.runOnce(()->{
      navx.reset();
    });
  }

  public void stopMotors() {
    for (int i = 0; i < modules.length; i++) {
      modules[i].setDriveVoltage(0);
      modules[i].setTurnVoltage(0);
    }
  }

  /**
   * NEEDS TO BE INVERTED SINCE navx.getYaw() RETURNS A CLOCKWISE POSITIVE VALUE!!!
   */
  public double getYaw() {
    return -navx.getYaw();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return navx.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }


  public SwerveModuleState[] getStates() {
    SwerveModuleState[] states = new SwerveModuleState[modules.length];
    for (int i = 0; i < states.length; i++) {
        states[i] = this.modules[i].getState();
    }
    return states;
   }
}