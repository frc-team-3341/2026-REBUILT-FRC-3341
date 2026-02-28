package frc.robot.subsystems.Intake;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.robot.Configs;
import frc.robot.Constants.IntakeConstants;

public class IntakeIOSpark implements IntakeIO {
    
    SparkFlex intakeSpark;
    SparkFlex liftSpark;

    double intakeSpeed;
    
    public IntakeIOSpark() {
        intakeSpark = new SparkFlex(IntakeConstants.INTAKE_CAN_ID, MotorType.kBrushless);
        intakeSpark.configure(Configs.Intake.INTAKE_CONFIG, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
        liftSpark = new SparkFlex(IntakeConstants.LIFT_CAN_ID, MotorType.kBrushless);
        liftSpark.configure(Configs.Intake.INTAKE_CONFIG, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
    

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.intakeMotorConnected = true;
        inputs.liftMotorConnected = true;
        inputs.intakeWheelSpeed = intakeSpeed;
    }

    @Override
    public void setIntakeSpeed(double speed) {
        intakeSpeed = Math.abs(speed);

        intakeSpark.set(intakeSpeed);
        liftSpark.set(intakeSpeed);
    }

    @Override
    public void reverseIntake(double speed) {
        intakeSpeed = -Math.abs(speed);

        intakeSpark.set(intakeSpeed);
        liftSpark.set(intakeSpeed);
    }

    @Override
    public void stopIntake() {
        intakeSpark.set(0);
        liftSpark.set(0);
    }
}
