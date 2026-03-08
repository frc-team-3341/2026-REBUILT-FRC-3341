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
import frc.util.ShooterUtil;
import frc.robot.subsystems.Intake;


import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.path.PathConstraints;

public class RobotContainer {
  private final Vision vision = new Vision();
  private final DriveSubsystem swerve = new DriveSubsystem(vision);
  private Intake robotIntake;

  
  CommandXboxController driver_controller = new CommandXboxController(OIConstants.kDriverControllerPort);
  private final ShooterSubsystem shooter = new ShooterSubsystem(swerve);
  private final SwerveTeleop swerveTeleop = new SwerveTeleop(swerve, driver_controller);


  public RobotContainer() {
    // Configure the button bindings
    createIntake();

    // Configure the button bindings
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
    driver_controller.rightBumper().onTrue(robotIntake.intakeBall()).onFalse(robotIntake.stopIntake());
    driver_controller.leftBumper().onTrue(robotIntake.reverseIntakeBall()).onFalse(robotIntake.stopIntake());

    driver_controller.x().onTrue(shooter.backupShooting());
    driver_controller.rightTrigger().onTrue(shooter.feed()).onFalse(shooter.stopFeed());
    driver_controller.y().onTrue(shooter.stopFlywheel());
    driver_controller.leftTrigger().onTrue(shooter.backfeed()).onFalse(shooter.stopFeed());
    driver_controller.b().onTrue(shooter.incrementRPM());
    driver_controller.a().onTrue(shooter.decrementRPM());

    // driver_controller.rightBumper().onTrue(shooter.incrementRPM());
    // driver_controller.leftBumper().onTrue(shooter.decrementRPM());

    // --------------------------------------------------------------------
    //         AUTO TESTING BINDINGS (uncomment when testing auto)
    // --------------------------------------------------------------------------
    
    // // Emergency Cancel all commands - press B
    // driver_controller.b().onTrue(
    //     Commands.runOnce(() -> {
    //         System.out.println("EMERGENCY CANCEL");
    //         CommandScheduler.getInstance().cancelAll();
    //         swerve.stopMotors();
    //     })
    // );
    
    // // Pathfinding test - press A
    // driver_controller.a().onTrue(
    //     AutoBuilder.pathfindToPose(
    //         new Pose2d(1.5, 1.0, Rotation2d.fromDegrees(0)),
    //         new PathConstraints(
    //             0.3,  // max velocity
    //             0.0,  // max acceleration
    //             Units.degreesToRadians(90.0),
    //             Units.degreesToRadians(90.0)
    //         ),
    //         0.0  // end velocity
    //     )
    // );
    
    // // Reset Odoemtry to (0, 0) - Press Left Bumper
    // driver_controller.leftBumper().onTrue(
    //     swerve.resetOdo()
    // );
  }

  public void createIntake() {
    robotIntake = new Intake(); 

  }
    
  public Command getAutonomousCommand() {
    // return Commands.sequence(
    //     AutoBuilder.pathfindToPose(
    //         new Pose2d(2.0, 2.0, Rotation2d.fromDegrees(0)),
    //         new PathConstraints(0.5, 0.5, 
    //             Units.degreesToRadians(360.0), Units.degreesToRadians(360.0)),
    //         0.0
    //     )
    // );
    return null;
  }
  public double getDistanceToHub() {

    Pose2d robotPose = swerve.getPose();
                  
    double distanceToHub = ShooterUtil.getDistanceToHub(robotPose);
    return distanceToHub;
  }
}
