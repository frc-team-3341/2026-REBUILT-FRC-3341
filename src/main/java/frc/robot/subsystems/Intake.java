package frc.robot.subsystems;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DigitalInput;

public class Intake extends SubsystemBase{
    private SparkMax intakeWheel;
    private SparkMaxConfig intakeConfig;
    private DigitalInput beamBreak;
    
    public Intake(){
        //The motor we're using for the kitbot is kBrushed
        intakeWheel = new SparkMax(1, MotorType.kBrushed);
        intakeConfig = new SparkMaxConfig();

        //This is the beambreak sensor
        beamBreak = new DigitalInput(0);

        intakeConfig.closedLoop.pid(0.01, 0, 0);

        intakeWheel.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    }
    public boolean getBeamBreak(){
        return beamBreak.get();
    }

    public Command intakeBall() {
        return runOnce(
            () -> {
                intakeWheel.set(0.15);
        });
    }
    public Command stopIntake(){
        return runOnce(
            () -> {
                intakeWheel.set(0);
        });
    }

}
