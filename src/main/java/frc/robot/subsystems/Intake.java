package frc.robot.subsystems;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Intake extends SubsystemBase{
    private SparkFlex intakeWheel;
    private SparkFlexConfig intakeConfig;
    private DigitalInput beamBreak;
    private int counter;
    
    public Intake(){
        //The motor we're using for the kitbot is kBrushed
        intakeWheel = new SparkFlex(Constants.DriveConstants.intakeMotorCanId, MotorType.kBrushless);
        intakeConfig = new SparkFlexConfig();

        //This is the beambreak sensor that senses whether the beam is hitting it or not
        beamBreak = new DigitalInput(3);
        counter = 0;

        intakeConfig.closedLoop.pid(0.01, 0, 0);

        intakeWheel.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    }

    public boolean getBeamBreak(){
        return beamBreak.get();
    }

    @Override
    public void periodic() {
        //counter++;
        SmartDashboard.putBoolean("beambreak state", getBeamBreak());
        System.out.println(getBeamBreak());
    }

    public Command intakeBall() {
        return runOnce(
            () -> {
                intakeWheel.set(0.25);
        });
    }
    public Command stopIntake(){
        return runOnce(
            () -> {
                intakeWheel.set(0);
        });
    }

}
