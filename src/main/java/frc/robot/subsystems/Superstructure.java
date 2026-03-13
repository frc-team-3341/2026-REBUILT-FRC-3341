package frc.robot.subsystems;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.*;

public class Superstructure extends SubsystemBase {
    
    DriveSubsystem swerve;
    ShooterSubsystem shooter;
    Intake intake;

    SuperState currentSuperState;
    SuperState prevSuperState;

    IntakeState currentIntakeState;

    ShooterState currentShooterState;

    FeederState currentFeederState;

    SwerveState currentSwerveState;


    public Superstructure(DriveSubsystem swerve, ShooterSubsystem shooter, Intake intake) {
        this.swerve = swerve;
        this.shooter = shooter;
        this.intake = intake;

        currentSuperState = SuperState.IDLE;
        prevSuperState = SuperState.IDLE;
        currentIntakeState = IntakeState.IDLE;
        currentShooterState = ShooterState.IDLE;
        currentFeederState = FeederState.IDLE;
        currentSwerveState = SwerveState.MANUAL;
    }

    public Command setSuperState(SuperState newSuperState) {
        System.out.println(
            "Transitioning to the requested SuperState " + 
            newSuperState + " from the current SuperState " + currentSuperState);
            
        switch (newSuperState) {
            case IDLE:
                return Commands.runOnce(() -> {
                    prevSuperState = currentSuperState;
                    currentSuperState = newSuperState;
                })
                .andThen(
                    setIntakeState(IntakeState.IDLE),
                    setFeederState(FeederState.IDLE),
                    setShooterState(ShooterState.IDLE),
                    setSwerveState(SwerveState.MANUAL)
                );
            case INTAKING:
                switch (currentSuperState) {
                    case PASSING:
                    case REVERSE:
                    case ALIGNING_TOWER_LEFT:
                    case ALIGNING_TOWER_RIGHT:
                        return setSuperState(currentSuperState); 
                    case IDLE:
                    case INTAKING:
                    case SCORING:
                        return Commands.runOnce(() -> {
                            prevSuperState = currentSuperState;
                            currentSuperState = newSuperState;
                        })
                        .andThen(
                            setIntakeState(IntakeState.INTAKE),
                            setSwerveState(SwerveState.MANUAL),
                            setShooterState(currentShooterState)
                        );
                    
                }
            case SCORING:
                switch (currentSuperState) {
                    case IDLE:
                    case INTAKING:
                    case SCORING:
                    case PASSING:
                    case REVERSE:
                        return Commands.runOnce(() -> {
                            prevSuperState = currentSuperState;
                            currentSuperState = newSuperState;
                        })
                        .andThen(
                            setIntakeState(IntakeState.IDLE),
                            setSwerveState(SwerveState.TRACKING_HUB),
                            setShooterState(ShooterState.SCORING)
                        );
                    case ALIGNING_TOWER_LEFT:
                    case ALIGNING_TOWER_RIGHT:
                        return setSuperState(currentSuperState);
                }
            case PASSING:
                switch (currentSuperState) {
                    case INTAKING:
                    case ALIGNING_TOWER_LEFT:
                    case ALIGNING_TOWER_RIGHT:
                        return setSuperState(currentSuperState);
                    case IDLE:
                    case SCORING:
                    case PASSING:
                    case REVERSE:
                        return Commands.runOnce(() -> {
                            prevSuperState = currentSuperState;
                            currentSuperState = newSuperState;
                        })
                        .andThen(
                            setIntakeState(IntakeState.INTAKE),
                            setSwerveState(SwerveState.MANUAL),
                            setShooterState(ShooterState.PASSING)
                        );
                    
                }
            case REVERSE:
                switch (currentSuperState) {
                    case INTAKING:
                    case ALIGNING_TOWER_LEFT:
                    case ALIGNING_TOWER_RIGHT:
                        return setSuperState(currentSuperState);
                    case IDLE:
                    case SCORING:
                    case PASSING:
                    case REVERSE:
                        return Commands.runOnce(() -> {
                            prevSuperState = currentSuperState;
                            currentSuperState = newSuperState;
                        })
                        .andThen(
                            setIntakeState(IntakeState.OUTTAKE),
                            setSwerveState(SwerveState.MANUAL),
                            setShooterState(currentShooterState)
                        );
                }
            case ALIGNING_TOWER_LEFT:
                switch (currentSuperState) {
                    case INTAKING:
                    case REVERSE:
                        return setSuperState(currentSuperState);
                    case SCORING:
                    case PASSING:
                    case IDLE:
                    case ALIGNING_TOWER_LEFT:    
                    case ALIGNING_TOWER_RIGHT:
                        return Commands.runOnce(() -> {
                            prevSuperState = currentSuperState;
                            currentSuperState = newSuperState;
                        })
                        .andThen(
                            setIntakeState(IntakeState.IDLE),
                            setFeederState(FeederState.IDLE),
                            setShooterState(ShooterState.IDLE),
                            setSwerveState(SwerveState.ALIGNING_TOWER_LEFT)
                        );
                    
                }
            case ALIGNING_TOWER_RIGHT:
                switch (currentSuperState) {
                    case INTAKING:
                    case REVERSE:
                        return setSuperState(currentSuperState);
                    case SCORING:
                    case PASSING:
                    case IDLE:
                    case ALIGNING_TOWER_LEFT:    
                    case ALIGNING_TOWER_RIGHT:
                        return Commands.runOnce(() -> {
                            prevSuperState = currentSuperState;
                            currentSuperState = newSuperState;
                        })
                        .andThen(
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
        return currentSuperState;
    }

    public SuperState getPreviousSuperState() {
        return prevSuperState;
    }

    public Command setIntakeState(IntakeState intakeState) {
        return Commands.runOnce(() -> this.currentIntakeState = intakeState)
            .andThen(intake.handleIntakeTransition(intakeState));
    }

    public Command setFeederState(FeederState feederState) {
        return Commands.runOnce(() -> this.currentFeederState = feederState)
            .andThen(shooter.handleFeederTransition(feederState));
    }

    public Command setShooterState(ShooterState shooterState) {
        return Commands.runOnce(() -> this.currentShooterState = shooterState)
            .andThen(shooter.handleShooterTransitions(shooterState));
    }

    public Command setSwerveState(SwerveState swerveState) {
        return Commands.runOnce(() -> this.currentSwerveState = swerveState)
            .andThen(swerve.handleSwerveTransitions(swerveState));
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