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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.Subsystem.*;

public class RobotContainer {
//Amogh Hongal was here first
  private final CommandXboxController joy1 = new CommandXboxController(0);
  private final Climber climber = new Climber();

  public RobotContainer() {
    configureButtonBindings();
  }

  // Button bindings
  private void configureButtonBindings() {
    joy1.povUp().whileTrue(climber.leadscrewUp()).onFalse(climber.leadscrewStop()); // d pad up control for going up
    joy1.povDown().whileTrue(climber.leadscrewDown()).onFalse(climber.leadscrewStop()); //d pad down control for going down
    joy1.a().onTrue(climber.servoOn()); // a to make the servo engage 
    joy1.b().onTrue(climber.servoOff()); // b to release the servo 
  }
//Ethan Dhakal was here
  public Command getAutonomousCommand() {
    return null;
  }
}