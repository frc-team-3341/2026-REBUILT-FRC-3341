// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.util.List;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */

public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();

  // The driver's controller
  private final CommandXboxController joy1 = new CommandXboxController(Constants.USBOrder.Zero);
  private Climber climber = new Climber();



  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    // m_robotDrive.setDefaultCommand(
    //     // The left stick controls translation of the robot.
    //     // Turning is controlled by the X axis of the right stick.
    //     new RunCommand(
    //         () -> m_robotDrive.drive(
    //             -MathUtil.applyDeadband(driver_controller.getLeftY(), OIConstants.kDriveDeadband),
    //             -MathUtil.applyDeadband(driver_controller.getLeftX(), OIConstants.kDriveDeadband),
    //             -MathUtil.applyDeadband(driver_controller.getRightX(), OIConstants.kDriveDeadband),
    //             true),
    //         m_robotDrive));
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
    new Trigger(() -> joy1.getLeftX() > 0.05).whileTrue(climber.leftMotorUp()).onFalse(climber.leftMotorStop());
    new Trigger(() -> joy1.getLeftX() < -0.05).whileTrue(climber.leftMotorDown()).onFalse(climber.leftMotorStop());
    new Trigger(() -> joy1.getRightX() > 0.05).whileTrue(climber.rightMotorUp()).onFalse(climber.rightMotorStop());
    new Trigger(() -> joy1.getRightX() < -0.05).whileTrue(climber.rightMotorUp()).onFalse(climber.rightMotorStop());
    // joy1.rightBumper().whileTrue(climber.leftMotorUp()).onFalse(climber.leftMotorStop());
    // joy1.leftBumper().whileTrue(climber.rightMotorDown()).onFalse(climber.rightMotorStop());
  }


  public Command getAutonomousCommand() {
    return null;
  }
}