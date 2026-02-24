package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Shooter.Shooter;

public class Superstructure extends SubsystemBase {
    
    Drive swerve;
    Shooter shooter;
    Intake intake;

    SuperState desiredSuperState;

    IntakeState desiredIntakeState;
    ShooterState desiredShooterState;
    FeederState desiredFeederState;

    public Superstructure(Drive swerve, Shooter shooter, Intake intake) {
        this.swerve = swerve;
        this.shooter = shooter;
        this.intake = intake;

        desiredSuperState = SuperState.IDLE;
        desiredIntakeState = IntakeState.IDLE;
        desiredShooterState = ShooterState.IDLE;
        desiredFeederState = FeederState.IDLE;
    }

    public Command setIntakeState(IntakeState desiredIntakeState) {
        return Commands.runOnce(() -> this.desiredIntakeState = desiredIntakeState)
            .alongWith(intake.handleIntakeTransition(desiredIntakeState));
    }

    public Command setFeederState(FeederState desiredFeederState) {
        return Commands.runOnce(() -> this.desiredFeederState = desiredFeederState)
            .alongWith(shooter.handleFeederTransition(desiredFeederState));
    }

    public enum SuperState {
        IDLE,
        INTAKING,
        SCORING,
        PASSING,
        REVERSE,
        ALIGNING_TOWER_LEFT,
        ALIGNING_TOWER_RIGHT
    }

    public enum IntakeState {
        IDLE,
        INTAKE,
        OUTTAKE
    }

    public enum ShooterState {
        IDLE,
        PASSING,
        SCORING
    }

    public enum FeederState {
        IDLE,
        FEED,
        BACKFEED
    }

    public enum SwerveState {
        MANUAL,
        TRACKING_HUB,
        ALIGNING_TOWER_LEFT,
        ALIGNING_TOWER_RIGHT
    }
}
