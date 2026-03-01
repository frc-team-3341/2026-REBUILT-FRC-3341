package frc.robot;


import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.SwerveTeleop;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Vision;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OIConstants;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Intake;


import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.path.PathConstraints;

public class RobotContainer {
  private final Vision vision = new Vision();
  private final DriveSubsystem swerve = new DriveSubsystem(vision);
  private Intake robotIntake;

  
  CommandXboxController driver_controller = new CommandXboxController(OIConstants.kDriverControllerPort);
  private final ShooterSubsystem shooter = new ShooterSubsystem();
  private final SwerveTeleop swerveTeleop = new SwerveTeleop(swerve, driver_controller);


  public RobotContainer() {
    // driver_controller.a().onTrue(shooter.incrementRPM());
    driver_controller.b().onTrue(shooter.stopMotor());
    // driver_controller.x().onTrue(shooter.decrementRPM());
    driver_controller.y().onChange(shooter.feed());
    driver_controller.rightBumper().onTrue(shooter.Score());
    // driver_controller.leftBumper().onTrue(shooter.backupShooting());
    //shooter.setRPM(shooter.getRPM4Vel(8.0));
  
    // Configure the button bindings
    createIntake();

    // Configure the button bindings
    configureButtonBindings();
    swerve.setDefaultCommand(swerveTeleop);
    
    // Warmup pathfinding (runs in background)
    PathfindingCommand.warmupCommand().schedule();
  }


  private void configureButtonBindings() {
    
    //change this based on what the driver wants for reset heading idk man
    driver_controller.rightStick().onTrue(swerve.zeroHeading());
    driver_controller.x().whileTrue(swerve.run(()->swerve.drive(1, 0, 0, false))  );

  }

  
  public void createIntake(){
    robotIntake = new Intake();

    /*
    driver_controller.a().onTrue(robotIntake.intakeBall()).onFalse(robotIntake.stopIntake());

    Trigger beamBreakIntake = new Trigger(() ->{
      return !robotIntake.getBeamBreak();
    });
    */

    // Trigger beamBreakInput = new Trigger(() -> {
      
    //   return robotIntake.getBeamBreak();
    // });
    

    // Trigger timer = new Trigger(() -> {
    //   if(!robotIntake.getBeamBreak()){
    //     robotIntake.addCounter(10);
    //   }
    //   else{
    //     robotIntake.setCounter(0);
    //   }

    //   if(robotIntake.getCounter() >= 1000){
    //     return false;
    //   }
    //   else{
    //     return true;
    //   }
    // });

    
    //Trigger motorOutputOn = (driver_controller.a()).and(timer);

    Trigger motorOutputOn = driver_controller.a();
    motorOutputOn.onTrue(robotIntake.intakeBall());

    Trigger motorBackwards = driver_controller.b();
    motorBackwards.onTrue(robotIntake.reverseIntakeBall()).onFalse(robotIntake.stopIntake());

   // motorOutputOn.toggleOnTrue(robotIntake.intakeBall());

    // Trigger keepOn = new Trigger(() -> {
    //   if(driver_controller.a().getAsBoolean()){
        
    //     if(robotIntake.getMotorOn()){
    //       robotIntake.setMotorOn(false);
    //     }
    //     else{
    //       robotIntake.setMotorOn(true);
    //     }
    //   }
    //   System.out.println(robotIntake.getMotorOn());
    //   return robotIntake.getMotorOn();
    // });

 //   timer.onFalse(robotIntake.stopIntake());
   // timer.onFalse(robotIntake.setMotorCommandOn(false));

    driver_controller.a().onTrue(robotIntake.keepOn());
    Trigger keepOn = new Trigger(() -> {
      
      return robotIntake.getMotorOn();
    });
   keepOn.toggleOnFalse(robotIntake.stopIntake());

  }
    // Zero Heading
    driver_controller.start().onTrue(
        Commands.runOnce(() -> {
            System.out.println("ZEROING HEADING");
        }).andThen(swerve.zeroHeading())
    );

    driver_controller.x().onTrue(swerve.zeroHeading());
    
    // Emergency Cancel all commands - press B
    driver_controller.b().onTrue(
        Commands.runOnce(() -> {
            System.out.println("EMERGENCY CANCEL");
            CommandScheduler.getInstance().cancelAll();
            swerve.stopMotors();
        })
    );
    
    // Pathfinding test - press A
    driver_controller.a().onTrue(
        AutoBuilder.pathfindToPose(
            new Pose2d(0.5, 0.5, Rotation2d.fromDegrees(0)),
            new PathConstraints(
                0.3,  // max velocity
                0.5,  // max acceleration
                Units.degreesToRadians(360.0),
                Units.degreesToRadians(360.0)
            ),
            0.0  // end velocity
        )
    );
    // driver_controller.a().onTrue(
    //     Commands.sequence(
    //         Commands.runOnce(() -> {
    //             Pose2d currentPose = swerve.getPose();
    //             System.out.println("===== PATHFINDING STARTED =====");
    //             System.out.println("Current Pose: " + currentPose);
                
    //             boolean isValidX = currentPose.getX() >= 0 && currentPose.getX() <= 16.54;
    //             boolean isValidY = currentPose.getY() >= 0 && currentPose.getY() <= 8.07;
                
    //             if (!isValidX || !isValidY) {
    //                 System.out.println("WARNING: Position is OFF FIELD!");
    //                 System.out.println("Auto-resetting to (1.0, 1.0, 0)");
    //                 swerve.resetOdometry(new Pose2d(1.0, 1.0, new Rotation2d(0)));
    //                 System.out.println("New pose: " + swerve.getPose());
    //             } else {
    //                 System.out.println("Position is valid - proceeding");
    //             }
    //         }),
            
    //         Commands.waitSeconds(0.1),
            
    //         Commands.runOnce(() -> {
    //             Pose2d current = swerve.getPose();
    //             Pose2d target = new Pose2d(0.5, 0.5, Rotation2d.fromDegrees(0));
    //             double distance = current.getTranslation().getDistance(target.getTranslation());
                
    //             System.out.println("Current Position: " + String.format("(%.2f, %.2f, %.2f )", 
    //                 current.getX(), current.getY(), current.getRotation().getDegrees()));
    //             System.out.println("Target Position:  (0.5, 0.5, 0.00 )");
    //             System.out.println("Distance to target: " + String.format("%.2f meters", distance));
    //             System.out.println("Creating pathfinding command...");
    //         }),
            
    //         AutoBuilder.pathfindToPose(
    //             new Pose2d(0.5, 0.5, Rotation2d.fromDegrees(0)),
    //             new PathConstraints(
    //                 0.7,  // max velocity
    //                 0.5,  // max acceleration
    //                 Units.degreesToRadians(360.0),
    //                 Units.degreesToRadians(360.0)
    //             ),
    //             0.0  // end velocity
    //         )
    //         .beforeStarting(() -> {
    //             System.out.println(">>> PATHFINDING COMMAND STARTING <<<");
    //         })
    //         .andThen(() -> {
    //             Pose2d finalPose = swerve.getPose();
    //             System.out.println(">>> PATHFINDING COMPLETED SUCCESSFULLY <<<");
    //             System.out.println("Final Position: " + String.format("(%.2f, %.2f, %.2f)", 
    //                 finalPose.getX(), finalPose.getY(), finalPose.getRotation().getDegrees()));
    //         })
    //         .finallyDo((interrupted) -> {
    //             if (interrupted) {
    //                 System.out.println("PATHFINDING INTERRUPTED");
    //             } else {
    //                 System.out.println("PATHFINDING ENDED NORMALLY");
    //             }
    //             swerve.stopMotors();
    //         })
    //         .withTimeout(15.0)  // 15 second timeout
    //     )
    // );
    
    // Drive Forward - press X
    driver_controller.x().onTrue(
        Commands.sequence(
            Commands.runOnce(() -> {
                System.out.println("DRIVE FORWARD TEST");
                System.out.println("Driving forward at 0.5 m/s for 2 seconds");
                System.out.println("Starting pose: " + swerve.getPose());
            }),
            Commands.run(() -> swerve.drive(0.5, 0, 0, false), swerve)
                .withTimeout(2.0),
            Commands.runOnce(() -> {
                swerve.stopMotors();
                System.out.println("Drive test complete");
                System.out.println("Final pose: " + swerve.getPose());
            })
        )
    );
    
    // Rotate - press Y
    driver_controller.y().onTrue(
        Commands.sequence(
            Commands.runOnce(() -> {
                System.out.println("ROTATION TEST");
                System.out.println("Rotating at 1.0 rad/s for 2 seconds");
                System.out.println("Starting heading: " + swerve.getPose().getRotation().getDegrees() + " ");
            }),
            Commands.run(() -> swerve.drive(0, 0, 1.0, false), swerve)
                .withTimeout(2.0),
            Commands.runOnce(() -> {
                swerve.stopMotors();
                System.out.println("Rotation test complete");
                System.out.println("Final heading: " + swerve.getPose().getRotation().getDegrees() + " ");
            })
        )
    );
    
    // Reset Odoemtry to (0, 0) - Press Left Bumper
    driver_controller.leftBumper().onTrue(
        swerve.resetOdo()
    );
  }

  public Command getAutonomousCommand() {
    return Commands.sequence(
        AutoBuilder.pathfindToPose(
            new Pose2d(2.0, 2.0, Rotation2d.fromDegrees(0)),
            new PathConstraints(0.5, 0.5, 
                Units.degreesToRadians(360.0), Units.degreesToRadians(360.0)),
            0.0
        )
    );
  }
}
