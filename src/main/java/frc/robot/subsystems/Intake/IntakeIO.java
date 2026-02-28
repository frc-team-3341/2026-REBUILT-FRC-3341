package frc.robot.subsystems.Intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
    @AutoLog
    public static class IntakeIOInputs {
        public boolean intakeMotorConnected = false;

        public boolean liftMotorConnected = false;

        public double intakeWheelSpeed = 0.0;
    }

    public default void updateInputs(IntakeIOInputs inputs) {}

    public default void setIntakeSpeed(double speed) {}

    public default void reverseIntake(double speed) {}

    public default void stopIntake() {}
}
