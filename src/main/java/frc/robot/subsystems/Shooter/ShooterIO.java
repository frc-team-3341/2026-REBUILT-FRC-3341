package frc.robot.subsystems.Shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
    @AutoLog
    public static class ShooterIOInputs {
        public boolean flywheelMotorConnected = false;

        public double targetFlywheelRPM = 0.0;

        public boolean feederMotorConnected = false;

        public double targetFeederRPM = 0.0;
    }
}
