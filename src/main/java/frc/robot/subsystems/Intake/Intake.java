package frc.robot.subsystems.Intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {

    IntakeIO io;
    IntakeIOInputsAutoLogged inputs;

    public Intake(IntakeIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
    }

    enum IntakeState {
        IDLE,
        INTAKE,
        OUTTAKE
    }
}
