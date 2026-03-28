package frc.robot;


import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.FeederState;
import frc.robot.subsystems.Superstructure.SuperState;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.ShootAuto;
import frc.robot.commands.SwerveTeleop;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Vision;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.events.EventTrigger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
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
  ShootAuto shootAuto;


  public RobotContainer() {

    shootAuto = new ShootAuto(shooter);

    NamedCommands.registerCommand("Shoot", shootAuto);
    NamedCommands.registerCommand("Start Lift", intake.startLift());

    swerveTeleop = new SwerveTeleop(swerve, driver_controller, swerve::aimDriveEnabled);

    superstructure = new Superstructure(swerve, shooter, intake);

    swerve.setDefaultCommand(swerveTeleop);

//     NamedCommands.registerCommand("Wait", new WaitCommand(5));

    configureButtonBindings();
    
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
        
    driver_controller.a().onTrue(PASSING);
        
    driver_controller.b().onTrue(IDLE);

    driver_controller.x().onTrue(SCORING);

    driver_controller.rightBumper().onTrue(INTAKING).onFalse(PREVIOUS);

    driver_controller.leftBumper().onTrue(REVERSE).onFalse(PREVIOUS_REVERSE);

//     driver_controller.povLeft().onTrue(ALIGNING_TOWER_LEFT);

//     driver_controller.povRight().onTrue(ALIGNING_TOWER_RIGHT);

    driver_controller.rightTrigger().onTrue(FEED).onFalse(Commands.runOnce(() -> CommandScheduler.getInstance().cancel(FEED)).alongWith(STOP_FEED));
//     driver_controller.rightTrigger().onTrue(FEED).onFalse(STOP_FEED);

    driver_controller.leftTrigger().onTrue(BACKFEED).onFalse(STOP_FEED_2);
       
    driver_controller.y().onTrue(shooter.backupShooting());


    new EventTrigger("ShootFromHere").onTrue(shootAuto);
//     new EventTrigger("Wait").onTrue(new WaitCommand(5));
//     new EventTrigger("Lift").onTrue(intake.startLift());
    
  }
    
  public Command getAutonomousCommand() {
//     return shootAuto.andThen(intake.startLift()).andThen(new ShootAuto(shooter));
      return swerve.getAutonomousCommand().andThen(new WaitCommand(4)).andThen(intake.startLift()).andThen(new ShootAuto(shooter));
  }

  public Superstructure getSuperstructure() {
    return superstructure;
  }
}
