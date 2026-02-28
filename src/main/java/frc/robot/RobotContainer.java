
// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;


import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.SwerveTeleop;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.FeederState;
import frc.robot.subsystems.Superstructure.SuperState;
import frc.robot.subsystems.Gyro.GyroIO;
import frc.robot.subsystems.Gyro.GyroIONavX;
import frc.robot.subsystems.Gyro.GyroIOSim;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Intake.IntakeIOSpark;
import frc.robot.subsystems.Modules.EasySwerveModuleIO;
import frc.robot.subsystems.Modules.EasySwerveModuleIOSim;
import frc.robot.subsystems.Modules.EasySwerveModuleIOSpark;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Shooter.ShooterIOSpark;
import frc.robot.subsystems.vision.*;
import frc.util.FuelSim;
import frc.util.ShooterUtil;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import frc.robot.Constants.Dimensions;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.VisionConstants;
import static frc.robot.Constants.ModeConstants.*;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // Subsystems
    private static Drive drive;
    private final Vision vision;
    private SwerveTeleop swerveTeleop;
    private SwerveDriveSimulation driveSimulation = null;

    private Superstructure superstructure;
    private Shooter shooter;
    private Intake intake;
    
    public FuelSim fuelsim;
    // Controller
    private final CommandXboxController controller = new CommandXboxController(0);
    // Dashboard inputs
    private final LoggedDashboardChooser<Command> autoChooser;

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        System.out.println(currentMode);
        switch (currentMode) {
            case REAL:
                // Real robot, instantiate hardware IO implementations
                drive = new Drive(
                        new GyroIONavX(),
                        new EasySwerveModuleIOSpark(0),
                        new EasySwerveModuleIOSpark(1),
                        new EasySwerveModuleIOSpark(2),
                        new EasySwerveModuleIOSpark(3),
                        (pose) -> {});

                shooter = new Shooter(new ShooterIOSpark());

                intake = new Intake(new IntakeIOSpark());
                        
                this.vision = new Vision(
                        drive,
                        new VisionIOPhotonVision(VisionConstants.CAMERA_NAMES[0], VisionConstants.CAMERA_TRANSFORMS[0]));

                break;
            case SIM:
                // create a maple-sim swerve drive simulation instance
                this.driveSimulation =
                        new SwerveDriveSimulation(Dimensions.mapleSimConfig, new Pose2d(3, 3, new Rotation2d()));
                // add the simulated drivetrain to the simulation field
        
                SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulation);
                // Sim robot, instantiate physics sim IO implementations
                drive = new Drive(
                        new GyroIOSim(driveSimulation.getGyroSimulation()),
                        new EasySwerveModuleIOSim(driveSimulation.getModules()[0]),
                        new EasySwerveModuleIOSim(driveSimulation.getModules()[1]),
                        new EasySwerveModuleIOSim(driveSimulation.getModules()[2]),
                        new EasySwerveModuleIOSim(driveSimulation.getModules()[3]),
                        driveSimulation::setSimulationWorldPose);

                vision = new Vision(
                        drive,
                        new VisionIOPhotonVisionSim(
                                VisionConstants.CAMERA_NAMES[0], VisionConstants.CAMERA_TRANSFORMS[0], driveSimulation::getSimulatedDriveTrainPose));

                configureFuelSim();
                configureFuelSimRobot();

                break;
            default:
                // Replayed robot, disable IO implementations
                drive = new Drive(
                        new GyroIO() {},
                        new EasySwerveModuleIO() {},
                        new EasySwerveModuleIO() {},
                        new EasySwerveModuleIO() {},
                        new EasySwerveModuleIO() {},
                        (pose) -> {});
                vision = new Vision(drive, new VisionIO() {}, new VisionIO() {});

                break;
        }

        swerveTeleop = new SwerveTeleop(drive, controller, drive::aimDriveEnabled);

        superstructure = new Superstructure(drive, shooter, intake);

        // Set up auto routines
        autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

       // Configure the button bindings
        configureButtonBindings();
        
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by instantiating a
     * {@link GenericHID} or one of its subclasses ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}),
     * and then passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        // Default command, normal field-relative drive
        drive.setDefaultCommand(swerveTeleop);

        controller.a().onTrue(
                superstructure.setSuperState(SuperState.PASSING)
        );
        
        controller.b().onTrue(
                superstructure.setSuperState(SuperState.IDLE)
        );

        controller.x().onTrue(
                superstructure.setSuperState(SuperState.SCORING)
        );

        controller.rightBumper().whileTrue(
                superstructure.setSuperState(SuperState.INTAKING)
        )
        .onFalse(
                superstructure.setSuperState(superstructure.getPreviousSuperState())
        );

        controller.leftBumper().whileTrue(
                superstructure.setSuperState(SuperState.REVERSE)
        )
        .onFalse(
                superstructure.setSuperState(superstructure.getPreviousSuperState())
        );

        controller.povLeft().onTrue(
                superstructure.setSuperState(SuperState.ALIGNING_TOWER_LEFT)
        );

        controller.povRight().onTrue(
                superstructure.setSuperState(SuperState.ALIGNING_TOWER_RIGHT)
        );

        controller.rightTrigger().onTrue(
                superstructure.setFeederState(FeederState.FEED)
        )
        .onFalse(
                superstructure.setFeederState(FeederState.IDLE)
        );



        // Reset gyro / odometry
        final Runnable resetGyro = Constants.ModeConstants.currentMode == Constants.ModeConstants.Mode.SIM
                ? () -> drive.resetOdometry(
                        driveSimulation
                                .getSimulatedDriveTrainPose()) // reset odometry to actual robot pose during simulation
                : () -> drive.resetOdometry(
                        new Pose2d(drive.getPose().getTranslation(), new Rotation2d())); // zero gyro
        controller.start().onTrue(Commands.runOnce(resetGyro, drive).ignoringDisable(true));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autoChooser.get();
    }

    private void configureFuelSim() {
    fuelsim = new FuelSim();

//     fuelsim.spawnStartingFuel();

    fuelsim.start();
    
    fuelsim.enableAirResistance(); // an additional drag force will be applied to fuel in physics update step
  


    SmartDashboard.putData(Commands.runOnce(() -> {
            fuelsim.clearFuel();
        //     fuelsim.spawnStartingFuel();

    })
    .withName("Reset Fuel")
    .ignoringDisable(true));


  }
   private void configureFuelSimRobot() {
        fuelsim.registerRobot(
                Dimensions.FULL_WIDTH.in(Meters),
                Dimensions.FULL_LENGTH.in(Meters),
                Dimensions.BUMPER_HEIGHT.in(Meters),
                drive::getPose,
                drive::getFieldSpeeds);
        
    }

    public void resetSimulationField() {
        if (Constants.ModeConstants.currentMode != Constants.ModeConstants.Mode.SIM) return;

        drive.resetOdometry(new Pose2d(3, 3, new Rotation2d()));
        SimulatedArena.getInstance().resetFieldForAuto();
    }

    public static Pose2d getPose() {
        return drive.getPose();
    }

    public void updateSimulation() {
        if (Constants.ModeConstants.currentMode != Constants.ModeConstants.Mode.SIM) return;

        SimulatedArena.getInstance().simulationPeriodic();
        Logger.recordOutput("FieldSimulation/RobotPosition", driveSimulation.getSimulatedDriveTrainPose());
        Logger.recordOutput(
                "FieldSimulation/Coral", SimulatedArena.getInstance().getGamePiecesArrayByType("Coral"));
        Logger.recordOutput(
                "FieldSimulation/Algae", SimulatedArena.getInstance().getGamePiecesArrayByType("Algae"));
    }
}
