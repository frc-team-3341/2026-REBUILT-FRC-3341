// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.SwerveTeleop;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OIConstants;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Intake;

import java.util.List;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */

public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem swerve = new DriveSubsystem();
  private Intake robotIntake;


  // The driver's controller
  CommandXboxController driver_controller = new CommandXboxController(OIConstants.kDriverControllerPort);
  private final ShooterSubsystem shooter = new ShooterSubsystem();
  private final SwerveTeleop swerveTeleop = new SwerveTeleop(swerve, driver_controller);


  public RobotContainer() {
  
    // Configure the button bindings
    createIntake();

    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    swerve.setDefaultCommand(swerveTeleop);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */

  private void configureButtonBindings() {
    
    //change this based on what the driver wants for reset heading idk man
    driver_controller.rightStick().onTrue(swerve.zeroHeading());

    driver_controller.a().onTrue(robotIntake.intakeBall()).onFalse(robotIntake.stopIntake());
    driver_controller.b().onTrue(robotIntake.reverseIntakeBall()).onFalse(robotIntake.stopIntake());

    driver_controller.x().onTrue(shooter.backupShooting());
    driver_controller.rightTrigger().onTrue(shooter.feed()).onFalse(shooter.stopFeed());
    driver_controller.y().onTrue(shooter.stopFlywheel());

  }

  
  public void createIntake(){
    robotIntake = new Intake(); 

  }

  public Command getAutonomousCommand() {
    return null;
  }
}
