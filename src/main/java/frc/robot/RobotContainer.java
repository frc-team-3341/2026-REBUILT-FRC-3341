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
import frc.robot.commands.SwerveTeleop;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
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
  CommandJoystick mech_joystick = new CommandJoystick(OIConstants.kMechJoystickPort);

  private final SwerveTeleop swerveTeleop = new SwerveTeleop(swerve, driver_controller);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
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
    driver_controller.y().onTrue(swerve.zeroHeading());
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

  public Command getAutonomousCommand() {
    return null;
  }
}