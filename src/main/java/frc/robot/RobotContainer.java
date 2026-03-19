package frc.robot;


import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.FeederState;
import frc.robot.subsystems.Superstructure.SuperState;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.SwerveTeleop;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Vision;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Intake;

public class RobotContainer {
  private final Vision vision = new Vision();
  private final DriveSubsystem swerve = new DriveSubsystem(vision);
  private Intake intake = new Intake();

  
  CommandXboxController driver_controller = new CommandXboxController(OIConstants.kDriverControllerPort);
  private final ShooterSubsystem shooter = new ShooterSubsystem(swerve, vision);
  private final SwerveTeleop swerveTeleop;
  private final Superstructure superstructure;


  public RobotContainer() {

    // Configure the button bindings
    configureButtonBindings();
    
    swerveTeleop = new SwerveTeleop(swerve, driver_controller, swerve::aimDriveEnabled);

    superstructure = new Superstructure(swerve, shooter, intake);

    swerve.setDefaultCommand(swerveTeleop);
    
    // Warmup pathfinding (runs in background)
    // PathfindingCommand.warmupCommand().schedule();
  }


  private void configureButtonBindings() {
    // Zero Heading
    driver_controller.start().onTrue(
        Commands.runOnce(() -> {
            System.out.println("ZEROING HEADING");
        }).andThen(swerve.zeroHeading())
    );

    Command IDLE = Commands.deferredProxy(
          () -> superstructure.setSuperState(SuperState.IDLE));

    Command INTAKING = Commands.deferredProxy(
          () -> superstructure.setSuperState(SuperState.INTAKING));

    Command PASSING = Commands.deferredProxy(
          () -> superstructure.setSuperState(SuperState.PASSING));

    Command SCORING = Commands.deferredProxy(
          () -> superstructure.setSuperState(SuperState.SCORING));

    Command REVERSE = Commands.deferredProxy(
          () -> superstructure.setSuperState(SuperState.REVERSE));

    Command ALIGNING_TOWER_LEFT = Commands.deferredProxy(
          () -> superstructure.setSuperState(SuperState.ALIGNING_TOWER_LEFT));

    Command ALIGNING_TOWER_RIGHT = Commands.deferredProxy(
          () -> superstructure.setSuperState(SuperState.ALIGNING_TOWER_RIGHT));

    Command PREVIOUS = Commands.deferredProxy(
          () -> superstructure.setSuperState(superstructure.getPreviousSuperState()));

    Command PREVIOUS_REVERSE = Commands.deferredProxy(
          () -> superstructure.setSuperState(superstructure.getPreviousSuperState()));

    Command FEED = Commands.deferredProxy(
          () -> superstructure.setFeederState(FeederState.FEED));

    Command BACKFEED = Commands.deferredProxy(
          () -> superstructure.setFeederState(FeederState.BACKFEED));

    Command STOP_FEED = Commands.deferredProxy(
          () -> superstructure.setFeederState(FeederState.IDLE));

    Command STOP_FEED_2 = Commands.deferredProxy(
          () -> superstructure.setFeederState(FeederState.IDLE));
        
//     driver_controller.a().onTrue(PASSING);
        
//     driver_controller.b().onTrue(IDLE);

//     driver_controller.x().onTrue(SCORING);

//     driver_controller.rightBumper().onTrue(INTAKING).onFalse(PREVIOUS);

//     driver_controller.leftBumper().onTrue(REVERSE).onFalse(PREVIOUS_REVERSE);

//     driver_controller.povLeft().onTrue(ALIGNING_TOWER_LEFT);

//     driver_controller.povRight().onTrue(ALIGNING_TOWER_RIGHT);

//     driver_controller.rightTrigger().onTrue(FEED).onFalse(STOP_FEED);

//     driver_controller.leftTrigger().onTrue(BACKFEED).onFalse(STOP_FEED_2);



    driver_controller.rightBumper().onTrue(Commands.runOnce(() -> intake.intake())).onFalse(Commands.runOnce(() -> intake.stopIntake()));
    driver_controller.leftBumper().onTrue(Commands.runOnce(() -> intake.reverseIntake())).onFalse(Commands.runOnce(() -> intake.stopIntake()));

    driver_controller.x().onTrue(shooter.backupShooting());
    driver_controller.rightTrigger().onTrue(Commands.runOnce(() -> shooter.feed())).onFalse(Commands.runOnce(() -> shooter.stopFeed()));
    driver_controller.y().onTrue(Commands.runOnce(() -> shooter.stopFlywheel()).alongWith(Commands.runOnce(() -> shooter.stopTopFeed())));
    driver_controller.leftTrigger().onTrue(Commands.runOnce(() -> shooter.backfeed())).onFalse(Commands.runOnce(() -> shooter.stopFeed()));
    driver_controller.b().onTrue(shooter.incrementRPM());
    driver_controller.a().onTrue(shooter.decrementRPM());

//     driver_controller.rightBumper().onTrue(shooter.incrementRPM());
//     driver_controller.leftBumper().onTrue(shooter.decrementRPM());

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
    
  public Command getAutonomousCommand() {
    // return Commands.sequence(
    //     AutoBuilder.pathfindToPose(
    //         new Pose2d(2.0, 2.0, Rotation2d.fromDegrees(0)),
    //         new PathConstraints(0.5, 0.5, 
    //             Units.degreesToRadians(360.0), Units.degreesToRadians(360.0)),
    //         0.0
    //     )
    // );
    return swerve.getAutonomousCommand();
  }
}
