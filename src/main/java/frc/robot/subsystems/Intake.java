package frc.robot.subsystems;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase{
    private SparkMax intakeWheel;
    private SparkMaxConfig intakeConfig;
    
    public Intake(){
        //The motor we're using for the kitbot is kBrushed
        intakeWheel = new SparkMax(CanID, MotorType.kBrushed);
        intakeConfig = new SparkMaxConfig();

        intakeConfig.closedLoop.pid(0.01, 0, 0);

        intakeWheel.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
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
