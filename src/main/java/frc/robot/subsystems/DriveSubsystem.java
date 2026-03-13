// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;
import static edu.wpi.first.units.Units.*;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Robot;
import frc.robot.subsystems.Superstructure.SwerveState;
import frc.util.ShooterUtil;

import static frc.util.ClimberUtil.getTowerPoses;

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
  private final StructPublisher<Pose2d> odometryPublisher;

  // Create kinematics object
  private SwerveDriveKinematics kinematics;
  private SwerveDrivePoseEstimator poseEstimator;
  private Vision vision;
  private boolean aimDriveEnabled;

  PIDController aimDriveController = new PIDController(0.1, 0, 0.05);
  
  private SwerveDriveSimulation mapleSimDrive;
  private final Field2d field = new Field2d();

  private SendableChooser<Command> autoChooser = new SendableChooser<>();
  // The gyro sensor
  private AHRS navx = new AHRS(NavXComType.kMXP_SPI);

  // Odometry class for tracking robot pose
  SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(
      DriveConstants.kDriveKinematics,
      Rotation2d.fromDegrees(getYaw()),
      new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
      });

// Add this method to DriveSubsystem
private void createSimulationSwerve(Pose2d startingPose) {
    DriveTrainSimulationConfig simulationConfig = DriveTrainSimulationConfig.Default()
        .withBumperSize(
            Meters.of(DriveConstants.kTrackWidth).plus(Inches.of(5)),
            Meters.of(DriveConstants.kWheelBase).plus(Inches.of(5))
        )
        .withRobotMass(Kilograms.of(DriveConstants.kRobotMassKg)) 
        .withCustomModuleTranslations(new Translation2d[] {
            new Translation2d(DriveConstants.kWheelBase / 2, DriveConstants.kTrackWidth / 2),
            new Translation2d(DriveConstants.kWheelBase / 2, -DriveConstants.kTrackWidth / 2),
            new Translation2d(-DriveConstants.kWheelBase / 2, DriveConstants.kTrackWidth / 2),
            new Translation2d(-DriveConstants.kWheelBase / 2, -DriveConstants.kTrackWidth / 2)
        })
        .withGyro(COTS.ofNav2X())
        .withSwerveModule(new SwerveModuleSimulationConfig(
            DCMotor.getNEO(1),  // Driving motor
            DCMotor.getNEO(1),  // Turning motor
            ModuleConstants.kDrivingMotorReduction,
            1.0,
            Volts.of(0.02),
            Volts.of(0.03),
            Inches.of(ModuleConstants.kWheelDiameterMeters * 39.3701 / 2), // Convert meters to inches radius
            KilogramSquareMeters.of(0.02),
            DriveConstants.kWheelCOF 
        ));

    // mapleSimDrive = new SwerveDriveSimulation(simulationConfig, startingPose);

    // Register the drivetrain simulation
    // SimulatedArena.getInstance().addDriveTrainSimulation(mapleSimDrive);
}


  /** Creates a new DriveSubsystem. */
  public DriveSubsystem(Vision vision) {
    modules = new EasySwerveModule[4];

    modules[0] = m_frontLeft;
    modules[1] = m_frontRight;
    modules[2] = m_rearLeft;
    modules[3] = m_rearRight;

    statePublisher = NetworkTableInstance.getDefault()
        .getStructArrayTopic("/SwerveStates", SwerveModuleState.struct).publish();
    poseEstimatorPublisher = NetworkTableInstance.getDefault()
        .getStructTopic("/PoseEstimator/EstimatedPose", Pose2d.struct).publish();
    odometryPublisher = NetworkTableInstance.getDefault()
        .getStructTopic("/PoseEstimator/OdometryPose", Pose2d.struct).publish();

    this.kinematics = Constants.DriveConstants.kDriveKinematics;
    this.vision = vision;
    
    // Initialize pose estimator with starting position
    this.poseEstimator = new SwerveDrivePoseEstimator(
        kinematics,
        Rotation2d.fromDegrees(this.getYaw()),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        },
        new Pose2d(1.190, 3.739, new Rotation2d()) // STARTING POSE 
    );
    
    SmartDashboard.putData("Field", field);

    createAuto();

    aimDriveEnabled = false;

    //this is required to stop the robot from tweaking when going from -179 to 179
    aimDriveController.enableContinuousInput(-180, 180);

    //Set tolerance to prevent pid controller oscillation
    aimDriveController.setTolerance(1.0);
        
    // Initialize simulation if in simulation mode
    if (Robot.isSimulation()) {
        createSimulationSwerve(new Pose2d()); // Or whatever starting pose you want
    }
  }

  private void createAuto()  {
        try {
          RobotConfig config = RobotConfig.fromGUISettings();
          // Configure AutoBuilder last
          AutoBuilder.configure(
              this::getPose, // Robot pose supplier
              this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
              this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
              this::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
              new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                      new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                      new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
              ),
              config, // The robot configuration
              () -> {
                // Boolean supplier that controls when the path will be mirrored for the red alliance
                // This will flip the path being followed to the red side of the field.
                // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                var alliance = DriverStation.getAlliance();
                if (alliance.isPresent()) {
                  return alliance.get() == DriverStation.Alliance.Red;
                }
                return false;
              },
              this); // Reference to this subsystem to set requirements        
          
          // Put in the name of the auto here
          autoChooser = AutoBuilder.buildAutoChooser("testIntakeAuto"); //TODO: put name of auto - use as parameter (String)
          SmartDashboard.putData(autoChooser);
        } catch (Exception e) {
          //If an exception is thrown here we are really in trouble
          e.printStackTrace();
          System.out.println("uh oh auto is really broken");
        }  
    }

  public Command getAutonomousCommand() {
      return autoChooser.getSelected();
  }

  //might want to move this into a separate file in the future
    public Command handleSwerveTransitions(SwerveState desiredState) {
        switch (desiredState) {
            case MANUAL:
                return Commands.runOnce(() -> aimDriveEnabled = false);

            case TRACKING_HUB:
                return Commands.runOnce(() -> aimDriveEnabled = true);

            case ALIGNING_TOWER_LEFT:
                return getTowerPoses() != null ? AutoBuilder.pathfindToPose(
                    getTowerPoses()[0], AutoConstants.PATH_CONSTRAINTS, 0)
                    : Commands.print("No alliance selected, cannot run alignment command!")
                    .alongWith(handleSwerveTransitions(SwerveState.MANUAL));

            case ALIGNING_TOWER_RIGHT:
                return getTowerPoses() != null ? AutoBuilder.pathfindToPose(
                    getTowerPoses()[1], AutoConstants.PATH_CONSTRAINTS, 0)
                    : Commands.print("No alliance selected, cannot run alignment command!")
                    .alongWith(handleSwerveTransitions(SwerveState.MANUAL));

            default:
                return Commands.print("Invalid Swerve State Provided!");
        }
    }
  
  public boolean aimDriveEnabled() {
    return aimDriveEnabled;
  }

  public PathConstraints getPathFindConstraints(){
    // Create path constraints
    PathConstraints constraints = new PathConstraints(
        0.4,   // maxVelocityMps
        0.1,   // maxAccelerationMpsSq
        Units.degreesToRadians(90.0),
        Units.degreesToRadians(90.0)
    );
    
    return constraints;
  }

  @Override
  public void simulationPeriodic() {
    // Update vision simulation with current pose
    if (Robot.isSimulation()) {
      vision.simulationPeriodic(getPose());
    }
  }

  @Override
  public void periodic() {
    // Update the odometry (wheel encoders + gyro only, no vision)
    m_odometry.update(
        Rotation2d.fromDegrees(getYaw()),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        });

    // Update the pose estimator with wheel odometry
    poseEstimator.update(
        Rotation2d.fromDegrees(getYaw()),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        }
    );

    // // Add vision measurements to pose estimator
    var intakeVisionEst = vision.getIntakeEstimatedGlobalPose();
    
    intakeVisionEst.ifPresent(est -> {
        // Get the standard deviations based on the quality of the vision estimate
        var estStdDevs = vision.getEstimationStdDevs();
        
        // Only add the measurement if std devs are not maxed out (which means rejected)
        if (estStdDevs.get(0, 0) < Double.MAX_VALUE) {
            poseEstimator.addVisionMeasurement(
                est.estimatedPose.toPose2d(), 
                est.timestampSeconds, 
                estStdDevs
            );
            
            // Debug: Vision measurement was accepted
            SmartDashboard.putBoolean("Vision/MeasurementAccepted", true);
        } else {
            // Debug: Vision measurement was rejected
            SmartDashboard.putBoolean("Vision/MeasurementAccepted", false);
        }
    });
    
    // If no vision estimate, mark as not accepted
    if (intakeVisionEst.isEmpty()) {
        SmartDashboard.putBoolean("Vision/MeasurementAccepted", false);
    }

    var batteryVisionEst = vision.getBatteryEstimatedGlobalPose();
    
    batteryVisionEst.ifPresent(est -> {
        // Get the standard deviations based on the quality of the vision estimate
        var estStdDevs = vision.getEstimationStdDevs();
        
        // Only add the measurement if std devs are not maxed out (which means rejected)
        if (estStdDevs.get(0, 0) < Double.MAX_VALUE) {
            poseEstimator.addVisionMeasurement(
                est.estimatedPose.toPose2d(), 
                est.timestampSeconds, 
                estStdDevs
            );
            
            // Debug: Vision measurement was accepted
            SmartDashboard.putBoolean("Vision/MeasurementAccepted", true);
        } else {
            // Debug: Vision measurement was rejected
            SmartDashboard.putBoolean("Vision/MeasurementAccepted", false);
        }
    });
    
    // If no vision estimate, mark as not accepted
    if (batteryVisionEst.isEmpty()) {
        SmartDashboard.putBoolean("Vision/MeasurementAccepted", false);
    }
  
    var shooterVisionEst = vision.getShooterEstimatedGlobalPose();
    
    shooterVisionEst.ifPresent(est -> {
        // Get the standard deviations based on the quality of the vision estimate
        var estStdDevs = vision.getEstimationStdDevs();
        
        // Only add the measurement if std devs are not maxed out (which means rejected)
        if (estStdDevs.get(0, 0) < Double.MAX_VALUE) {
            poseEstimator.addVisionMeasurement(
                est.estimatedPose.toPose2d(), 
                est.timestampSeconds, 
                estStdDevs
            );
            
            // Debug: Vision measurement was accepted
            SmartDashboard.putBoolean("Vision/MeasurementAccepted", true);
        } else {
            // Debug: Vision measurement was rejected
            SmartDashboard.putBoolean("Vision/MeasurementAccepted", false);
        }
    });
    
    // If no vision estimate, mark as not accepted
    if (shooterVisionEst.isEmpty()) {
        SmartDashboard.putBoolean("Vision/MeasurementAccepted", false);
    }

    // Odometry pose (wheel encoders + gyro only)
    Pose2d odometryPose = m_odometry.getPoseMeters();
    SmartDashboard.putString("Odometry/Pose", 
        String.format("(%.2f, %.2f, %.2f°)", 
            odometryPose.getX(),
            odometryPose.getY(),
            odometryPose.getRotation().getDegrees()));
    SmartDashboard.putNumber("Odometry/X", odometryPose.getX());
    SmartDashboard.putNumber("Odometry/Y", odometryPose.getY());
    SmartDashboard.putNumber("Odometry/Rotation", odometryPose.getRotation().getDegrees());
    
    // Combined pose (odometry + vision)
    Pose2d estimatedPose = poseEstimator.getEstimatedPosition();
    SmartDashboard.putString("PoseEstimator/CombinedPose", 
        String.format("(%.2f, %.2f, %.2f°)", 
            estimatedPose.getX(),
            estimatedPose.getY(),
            estimatedPose.getRotation().getDegrees()));
    SmartDashboard.putNumber("PoseEstimator/X", estimatedPose.getX());
    SmartDashboard.putNumber("PoseEstimator/Y", estimatedPose.getY());
    SmartDashboard.putNumber("PoseEstimator/Rotation", estimatedPose.getRotation().getDegrees());
    
    // Error between odometry and combined estimate
    double xError = estimatedPose.getX() - odometryPose.getX();
    double yError = estimatedPose.getY() - odometryPose.getY();
    double totalError = Math.sqrt(xError * xError + yError * yError);
    SmartDashboard.putNumber("PoseEstimator/ErrorFromOdometry", totalError);
    
    // NavX data
    SmartDashboard.putNumber("NavX/Yaw", this.getYaw());
    SmartDashboard.putNumber("NavX/Angle", navx.getAngle());
    
    // Update field widget
    field.setRobotPose(estimatedPose);
    
    // Publish to NetworkTables for AdvantageScope or other tools
    // poseEstimatorPublisher.set(estimatedPose);
    odometryPublisher.set(odometryPose);
  }

  //---------------METHODS----------------
  /**
   * Drive the robot for PathPlannerLib using robot-relative chassis speeds
   * @param speeds Robot-relative ChassisSpeeds
   */
  public void driveRobotRelative(ChassisSpeeds speeds) {
    drive(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond, false); //NOT field relative - it's robot relative
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

      public void aimDrive(double xSpeed, double ySpeed) { 

      Pose2d currentPose = getPose();

      Translation2d hubCenterPose = ShooterUtil.getHubTranslation2d();

      //end method if there is no alliance selected cuz this should only be used on the field
      if (hubCenterPose == null) {
        return;
      }
      
      double theta;

      double xDisplacement = (hubCenterPose.getX() - currentPose.getX()); 
      double yDisplacement = (hubCenterPose.getY() - currentPose.getY());

      theta = Math.toDegrees(Math.atan2(yDisplacement, xDisplacement)); 
      
      double rotOutput = aimDriveController.calculate(getRotation().getDegrees(), theta); 

      drive(xSpeed, ySpeed, rotOutput, true);
  }

  /**
   * Returns the currently-estimated pose of the robot (combined odometry + vision).
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }
  
  /**
   * Returns the odometry-only pose of the robot (no vision fusion).
   *
   * @return The odometry pose.
   */
  public Pose2d getOdometryPose() {
    return m_odometry.getPoseMeters();
  }
    
  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(
        Rotation2d.fromDegrees(this.getYaw()),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        },
        pose);
    
    poseEstimator.resetPosition(
        Rotation2d.fromDegrees(this.getYaw()),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        },
        pose);

    // if (Robot.isSimulation() && mapleSimDrive != null) {
    //   mapleSimDrive.setSimulationWorldPose(pose);
    //   vision.resetSimPose(pose);
    // }
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

    // mapleSimDrive.setSimulationWorldPose(new Pose2d(2, 2, new Rotation2d()));
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

  public Command resetOdo(){
    return this.runOnce(()->{
      this.resetOdometry(new Pose2d(0, 0, new Rotation2d(0)));
    });
  }

  //MUST BE NEGATIVE PLEASE DO NOT CHANGE
  public double getYaw() {
    return -navx.getYaw();
  }

  public void stopMotors() {
    for (int i = 0; i < modules.length; i++) {
      modules[i].setDriveVoltage(0);
      modules[i].setTurnVoltage(0);
    }
  }

  /** 
    * Get Rotation2d of Navx. Positive value (CCW positive default).
    */
   public Rotation2d getRotation() {
      return navx.getRotation2d();
   }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return navx.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

  /**
    * Get chassis speeds for PathPlannerLib
    */
   public ChassisSpeeds getRobotRelativeSpeeds() {
      return ChassisSpeeds.fromFieldRelativeSpeeds(kinematics.toChassisSpeeds(getActualStates()), getRotation());
   }

    /**
    * Gets the actual SwerveModuleState[] for our use in code
    */
   public SwerveModuleState[] getActualStates() {
      SwerveModuleState[] states = new SwerveModuleState[modules.length];
      for (int i = 0; i < states.length; i++) {
         states[i] = this.modules[i].getState();
      }
      return states;
   }
}