package frc.robot.subsystems.Shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
    @AutoLog
    public static class ShooterIOInputs {
        public boolean flywheelMotorConnected = false;

        public double flywheelRPM = 0.0;

        public double targetFlywheelRPM = 0.0;

        public boolean feederMotorConnected = false;

        public double feederSpeed = 0.0;
    }

    public default void updateInputs(ShooterIOInputs inputs) {}

    public default void setFlywheelRPM(double RPM) {}

    public default void setFeederSpeed(double speed) {}

    public default void reverseFeed(double speed) {}

    public default void stopFlywheel() {}

    public default void stopFeeder() {}
}
