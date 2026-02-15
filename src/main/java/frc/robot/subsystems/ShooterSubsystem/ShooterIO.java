package frc.robot.subsystems.ShooterSubsystem;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

public interface ShooterIO {
    public boolean turnMotorConnected = false;
    public Voltage turnAppliedVolts = Volts.zero();
    public Current turnCurrent = Amps.zero();
    public Angle turnPosition = Radians.zero();
    public AngularVelocity turnVelocity = RadiansPerSecond.zero();

    public boolean flywheelMotorConnected = false;
    public Voltage flywheelAppliedVolts = Volts.zero();
    public Current flywheelCurrent = Amps.zero();
    public AngularVelocity flywheelSpeed = RadiansPerSecond.zero();
    public AngularAcceleration flywheelAccel = RadiansPerSecondPerSecond.zero();
    public AngularVelocity flywheelSetpointSpeed = RadiansPerSecond.zero();
    public AngularAcceleration flywheelSetpointAccel = RadiansPerSecondPerSecond.zero();

    public default void updateInputs(){}

    public default void setShooterRPM(double RPM){}
    
    public default void setFeederRPM(double RPM){}
    
    public default double calculateLinearLaunchVelocity(double distance){
        return 0;
    }

    public default void setShooterPIDFF(
            double kP, double kD, double kV, double kA, double kS) {}

}