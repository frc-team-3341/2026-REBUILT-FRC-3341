// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OIConstants;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;


/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */

public class RobotContainer {
  // The robot's subsystems
  CommandXboxController driver_controller = new CommandXboxController(OIConstants.kDriverControllerPort);
  private final ShooterSubsystem shooter = new ShooterSubsystem();
  public RobotContainer() {
    driver_controller.a().onTrue(shooter.incrementVel());
    driver_controller.b().onTrue(shooter.stopMotor());
    driver_controller.x().onTrue(shooter.decrementVel());

  }

  public Command getAutonomousCommand() {
    return null;
  }
}
