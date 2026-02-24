package frc.robot.subsystems.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Superstructure.FeederState;
import frc.robot.subsystems.Superstructure.ShooterState;

import static frc.robot.Constants.ShooterConstants.*;

public class Shooter extends SubsystemBase {
    
    ShooterIO io;
    ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

    public Shooter(ShooterIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
    }

    public Command handleFeederTransition(FeederState desiredState) {
        switch (desiredState) {
            case IDLE:
                return this.runOnce(() -> io.stopFlywheel());

            case FEED:
                return this.runOnce(() -> io.setFeederSpeed(FEEDING_SPEED));

            case BACKFEED:
                return this.runOnce(() -> io.reverseFeed(BACKFEED_SPEED));
                
            default:
                return Commands.print("Invalid Feeder State Provided!");

        }
    }

    //TODO finish ts
    public Command handleShooterTransitions(ShooterState desiredState) {
        switch (desiredState) {
            default:
                return Commands.print("Invalid Shooter State Provided!");
        }
    }
}
