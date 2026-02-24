package frc.robot.subsystems.Shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
    
    ShooterIO io;

    public Shooter(ShooterIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {

    }

    enum ShooterState {
        IDLE,
        PASSING,
        SCORING
    }

    enum FeederState {
        IDLE,
        FEED,
        BACKFEED
    }
}
