package frc.robot.subsystems;

import org.littletonrobotics.junction.AutoLogOutput;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Shooter.Shooter;

public class Superstructure extends SubsystemBase {
    
    Drive swerve;
    Shooter shooter;
    Intake intake;

    @AutoLogOutput
    SuperState desiredSuperState;

    @AutoLogOutput
    SuperState prevSuperState;

    @AutoLogOutput
    IntakeState desiredIntakeState;

    @AutoLogOutput
    ShooterState desiredShooterState;

    @AutoLogOutput
    FeederState desiredFeederState;

    @AutoLogOutput
    SwerveState desiredSwerveState;


    public Superstructure(Drive swerve, Shooter shooter, Intake intake) {
        this.swerve = swerve;
        this.shooter = shooter;
        this.intake = intake;

        desiredSuperState = SuperState.IDLE;
        prevSuperState = SuperState.IDLE;
        desiredIntakeState = IntakeState.IDLE;
        desiredShooterState = ShooterState.IDLE;
        desiredFeederState = FeederState.IDLE;
        desiredSwerveState = SwerveState.MANUAL;
    }

    public Command setSuperState(SuperState newSuperState) {
        switch (newSuperState) {
            case IDLE:
                return Commands.runOnce(() -> {
                    prevSuperState = desiredSuperState;
                    desiredSuperState = newSuperState;
                })
                .alongWith(
                    setIntakeState(IntakeState.IDLE),
                    setFeederState(FeederState.IDLE),
                    setShooterState(ShooterState.IDLE),
                    setSwerveState(SwerveState.MANUAL)
                );
            case INTAKING:
                switch (desiredSuperState) {
                    case PASSING:
                    case REVERSE:
                    case ALIGNING_TOWER_LEFT:
                    case ALIGNING_TOWER_RIGHT:
                        return setSuperState(desiredSuperState);
                    case IDLE:
                    case INTAKING:
                    case SCORING:
                        return Commands.runOnce(() -> {
                            prevSuperState = desiredSuperState;
                            desiredSuperState = newSuperState;
                        })
                        .alongWith(
                            setIntakeState(IntakeState.INTAKE),
                            setSwerveState(SwerveState.MANUAL),
                            setShooterState(desiredShooterState)
                        );
                    
                }
            case SCORING:
                switch (desiredSuperState) {
                    case IDLE:
                    case INTAKING:
                    case SCORING:
                    case PASSING:
                    case REVERSE:
                        return Commands.runOnce(() -> {
                            prevSuperState = desiredSuperState;
                            desiredSuperState = newSuperState;
                        })
                        .alongWith(
                            setIntakeState(IntakeState.IDLE),
                            setSwerveState(SwerveState.TRACKING_HUB),
                            setShooterState(ShooterState.SCORING)
                        );
                    case ALIGNING_TOWER_LEFT:
                    case ALIGNING_TOWER_RIGHT:
                        return setSuperState(desiredSuperState);
                }
            case PASSING:
                switch (desiredSuperState) {
                    case INTAKING:
                    case ALIGNING_TOWER_LEFT:
                    case ALIGNING_TOWER_RIGHT:
                        return setSuperState(desiredSuperState);
                    case IDLE:
                    case SCORING:
                    case PASSING:
                    case REVERSE:
                        return Commands.runOnce(() -> {
                            prevSuperState = desiredSuperState;
                            desiredSuperState = newSuperState;
                        })
                        .alongWith(
                            setIntakeState(IntakeState.INTAKE),
                            setSwerveState(SwerveState.MANUAL),
                            setShooterState(ShooterState.PASSING)
                        );
                    
                }
            case REVERSE:
                switch (desiredSuperState) {
                    case INTAKING:
                    case ALIGNING_TOWER_LEFT:
                    case ALIGNING_TOWER_RIGHT:
                        return setSuperState(desiredSuperState);
                    case IDLE:
                    case SCORING:
                    case PASSING:
                    case REVERSE:
                        return Commands.runOnce(() -> {
                            prevSuperState = desiredSuperState;
                            desiredSuperState = newSuperState;
                        })
                        .alongWith(
                            setIntakeState(IntakeState.OUTTAKE),
                            setFeederState(FeederState.BACKFEED),
                            setSwerveState(SwerveState.MANUAL),
                            setShooterState(desiredShooterState)
                        );
                }
            case ALIGNING_TOWER_LEFT:
                switch (desiredSuperState) {
                    case INTAKING:
                    case REVERSE:
                        return setSuperState(desiredSuperState);
                    case SCORING:
                    case PASSING:
                    case IDLE:
                    case ALIGNING_TOWER_LEFT:    
                    case ALIGNING_TOWER_RIGHT:
                        return Commands.runOnce(() -> {
                            prevSuperState = desiredSuperState;
                            desiredSuperState = newSuperState;
                        })
                        .alongWith(
                            setIntakeState(IntakeState.IDLE),
                            setFeederState(FeederState.IDLE),
                            setShooterState(ShooterState.IDLE),
                            setSwerveState(SwerveState.ALIGNING_TOWER_LEFT)
                        );
                    
                }
            case ALIGNING_TOWER_RIGHT:
                switch (desiredSuperState) {
                    case INTAKING:
                    case REVERSE:
                        return setSuperState(desiredSuperState);
                    case SCORING:
                    case PASSING:
                    case IDLE:
                    case ALIGNING_TOWER_LEFT:    
                    case ALIGNING_TOWER_RIGHT:
                        return Commands.runOnce(() -> {
                            prevSuperState = desiredSuperState;
                            desiredSuperState = newSuperState;
                        })
                        .alongWith(
                            setIntakeState(IntakeState.IDLE),
                            setFeederState(FeederState.IDLE),
                            setShooterState(ShooterState.IDLE),
                            setSwerveState(SwerveState.ALIGNING_TOWER_RIGHT)
                        );
                    
                }
            default:
                return Commands.print("Invalid SuperState provided!");
        }
    }

    public SuperState getCurrentSuperState() {
        return desiredSuperState;
    }

    public SuperState getPreviousSuperState() {
        return prevSuperState;
    }

    public Command setIntakeState(IntakeState intakeState) {
        return Commands.runOnce(() -> this.desiredIntakeState = intakeState)
            .alongWith(intake.handleIntakeTransition(intakeState));
    }

    public Command setFeederState(FeederState feederState) {
        return Commands.runOnce(() -> this.desiredFeederState = feederState)
            .alongWith(shooter.handleFeederTransition(feederState));
    }

    public Command setShooterState(ShooterState shooterState) {
        return Commands.runOnce(() -> this.desiredShooterState = shooterState)
            .alongWith(shooter.handleShooterTransitions(shooterState));
    }

    public Command setSwerveState(SwerveState swerveState) {
        return Commands.runOnce(() -> this.desiredSwerveState = swerveState)
            .alongWith(swerve.handleSwerveTransitions(swerveState));
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
