package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.commands.SwerveTeleop;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.vision.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import static frc.robot.Constants.VisionConstants.CAMERA_TRANSFORMS;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.path.PathConstraints;

public class RobotContainer {
  private final VisionIOPhotonVision vision = new VisionIOPhotonVision(VisionConstants.CAMERA_NAMES[0], VisionConstants.CAMERA_TRANSFORMS[0]);
  private final DriveSubsystem swerve = new DriveSubsystem(vision);
  
  CommandXboxController driver_controller = new CommandXboxController(OIConstants.kDriverControllerPort);
  CommandJoystick mech_joystick = new CommandJoystick(OIConstants.kMechJoystickPort);

  private final SwerveTeleop swerveTeleop = new SwerveTeleop(swerve, driver_controller);

  public RobotContainer() {
    configureButtonBindings();
    swerve.setDefaultCommand(swerveTeleop);
    
    // Warmup pathfinding (runs in background)
    PathfindingCommand.warmupCommand().schedule();
  }

  private void configureButtonBindings() {
    // Zero Heading
    driver_controller.start().onTrue(
        Commands.runOnce(() -> {
            System.out.println("ZEROING HEADING");
        }).andThen(swerve.zeroHeading())
    );
    
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
        Commands.sequence(
            Commands.runOnce(() -> {
                Pose2d currentPose = swerve.getPose();
                System.out.println("===== PATHFINDING STARTED =====");
                System.out.println("Current Pose: " + currentPose);
                
                boolean isValidX = currentPose.getX() >= 0 && currentPose.getX() <= 16.54;
                boolean isValidY = currentPose.getY() >= 0 && currentPose.getY() <= 8.07;
                
                if (!isValidX || !isValidY) {
                    System.out.println("WARNING: Position is OFF FIELD!");
                    System.out.println("Auto-resetting to (1.0, 1.0, 0)");
                    swerve.resetOdometry(new Pose2d(1.0, 1.0, new Rotation2d(0)));
                    System.out.println("New pose: " + swerve.getPose());
                } else {
                    System.out.println("Position is valid - proceeding");
                }
            }),
            
            Commands.waitSeconds(0.1),
            
            Commands.runOnce(() -> {
                Pose2d current = swerve.getPose();
                Pose2d target = new Pose2d(0.5, 0.5, Rotation2d.fromDegrees(0));
                double distance = current.getTranslation().getDistance(target.getTranslation());
                
                System.out.println("Current Position: " + String.format("(%.2f, %.2f, %.2f )", 
                    current.getX(), current.getY(), current.getRotation().getDegrees()));
                System.out.println("Target Position:  (0.5, 0.5, 0.00 )");
                System.out.println("Distance to target: " + String.format("%.2f meters", distance));
                System.out.println("Creating pathfinding command...");
            }),
            
            AutoBuilder.pathfindToPose(
                new Pose2d(0.5, 0.5, Rotation2d.fromDegrees(0)),
                new PathConstraints(
                    0.7,  // max velocity
                    0.5,  // max acceleration
                    Units.degreesToRadians(360.0),
                    Units.degreesToRadians(360.0)
                ),
                0.0  // end velocity
            )
            .beforeStarting(() -> {
                System.out.println(">>> PATHFINDING COMMAND STARTING <<<");
            })
            .andThen(() -> {
                Pose2d finalPose = swerve.getPose();
                System.out.println(">>> PATHFINDING COMPLETED SUCCESSFULLY <<<");
                System.out.println("Final Position: " + String.format("(%.2f, %.2f, %.2f)", 
                    finalPose.getX(), finalPose.getY(), finalPose.getRotation().getDegrees()));
            })
            .finallyDo((interrupted) -> {
                if (interrupted) {
                    System.out.println("PATHFINDING INTERRUPTED");
                } else {
                    System.out.println("PATHFINDING ENDED NORMALLY");
                }
                swerve.stopMotors();
            })
            .withTimeout(15.0)  // 15 second timeout
        )
    );
    
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