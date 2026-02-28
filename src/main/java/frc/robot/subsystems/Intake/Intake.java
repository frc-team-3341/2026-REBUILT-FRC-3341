package frc.robot.subsystems.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Superstructure.IntakeState;

import static frc.robot.Constants.IntakeConstants.*;

public class Intake extends SubsystemBase {

    IntakeIO io;
    IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

    public Intake(IntakeIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
    }

    public Command handleIntakeTransition(IntakeState desiredState) {
        switch (desiredState) {
            case IDLE:
                return this.runOnce(() -> io.stopIntake());

            case INTAKE:
                return this.runOnce(() -> io.setIntakeSpeed(INTAKE_SPEED));

            case OUTTAKE:
                return this.runOnce(() -> io.reverseIntake(OUTTAKE_SPEED));

            default:
                return Commands.print("Invalid Intake State Provided!");

        }
    }
}
