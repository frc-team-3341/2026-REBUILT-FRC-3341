package frc.robot.subsystems.Intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {

    IntakeIO io;

    public Intake(IntakeIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {

    }

    enum IntakeState {
        IDLE,
        INTAKE,
        OUTTAKE
    }
}
