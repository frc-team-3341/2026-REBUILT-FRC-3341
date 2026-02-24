package frc.robot.subsystems.Intake;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.robot.Configs;
import frc.robot.Constants.IntakeConstants;

public class IntakeIOSpark implements IntakeIO {
    
    SparkFlex intakeSpark;
    
    public IntakeIOSpark() {
        intakeSpark = new SparkFlex(IntakeConstants.INTAKE_CAN_ID, MotorType.kBrushless);

        intakeSpark.configure(Configs.Intake.INTAKE_CONFIG, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {

    }

    @Override
    public void setIntakeSpeed(double speed) {
        intakeSpark.set(Math.abs(speed));
    }

    @Override
    public void reverseIntake(double speed) {
        intakeSpark.set(-Math.abs(speed));
    }

    @Override
    public void stopIntake() {
        intakeSpark.set(0);
    }
}
