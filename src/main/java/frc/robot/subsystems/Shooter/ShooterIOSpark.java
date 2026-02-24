package frc.robot.subsystems.Shooter;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.robot.Configs;
import frc.robot.Constants.ShooterConstants;

public class ShooterIOSpark implements ShooterIO {
    
    SparkFlex flywheelSpark;
    SparkFlex feederSpark;

    SparkClosedLoopController flywheelPID;
    RelativeEncoder shooterEncoder;

    double setpointRPM;
    double setpointSpeed;

    public ShooterIOSpark() {
        flywheelSpark = new SparkFlex(ShooterConstants.SHOOTER_FLYWHEEL_CAN_ID, MotorType.kBrushless);
        feederSpark = new SparkFlex(ShooterConstants.FEEDER_CAN_ID, MotorType.kBrushless);

        flywheelSpark.configure(Configs.Shooter.FLYWHEEL_CONFIG, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        feederSpark.configure(Configs.Shooter.FEEDER_CONFIG, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        flywheelPID = flywheelSpark.getClosedLoopController();
        shooterEncoder = flywheelSpark.getEncoder();
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        inputs.feederMotorConnected = true;
        inputs.flywheelMotorConnected = true;

        inputs.targetFlywheelRPM = setpointRPM;
        inputs.flywheelRPM = shooterEncoder.getVelocity();

        inputs.feederSpeed = setpointSpeed;
    }

    @Override
    public void setFlywheelRPM(double RPM) {
        this.setpointRPM = RPM;

        flywheelPID.setSetpoint(RPM, ControlType.kVelocity);
    }

    @Override
    public void setFeederSpeed(double speed) {
        setpointSpeed = Math.abs(speed);

        feederSpark.set(setpointSpeed);
    }

    @Override
    public void reverseFeed(double speed) {
        setpointSpeed = -Math.abs(speed);

        feederSpark.set(setpointSpeed);
    }

    @Override
    public void stopFlywheel() {
        flywheelPID.setSetpoint(0, ControlType.kVelocity);
    }

    @Override
    public void stopFeeder() {
        feederSpark.set(0);
    }
}
