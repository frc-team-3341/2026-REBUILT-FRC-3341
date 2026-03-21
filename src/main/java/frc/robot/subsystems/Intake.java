package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkFlex;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.subsystems.Superstructure.IntakeState;
import static frc.robot.Constants.IntakeConstants.*;

public class Intake extends SubsystemBase{
    private SparkFlex liftWheel;
    private SparkFlex intakeWheel;

    public Intake(){
        //The motor we're using for the kitbot is kBrushed
        liftWheel = new SparkFlex(LIFT_CAN_ID, MotorType.kBrushless);

        intakeWheel = new SparkFlex(INTAKE_CAN_ID, MotorType.kBrushless);
        

        intakeWheel.configure(Configs.Intake.INTAKE_CONFIG, com.revrobotics.ResetMode.kResetSafeParameters, com.revrobotics.PersistMode.kPersistParameters  );
        liftWheel.configure(Configs.Intake.LIFT_CONFIG, com.revrobotics.ResetMode.kResetSafeParameters, com.revrobotics.PersistMode.kPersistParameters);

    }

    @Override
    public void periodic() {
    
    }

    //fix this to make sure intake wheel is inverted
    public void intake() {
        liftWheel.set(LIFT_SPEED);
        intakeWheel.set(-INTAKE_SPEED);
    }

    //fix this to make sure intake wheel is inverted
    public void reverseIntake() {
        liftWheel.set(-LIFT_SPEED);
        intakeWheel.set(INTAKE_SPEED);
    }
    
    public void stopIntake(){
        liftWheel.set(0);
        intakeWheel.set(0);
    }

    public Command handleIntakeTransition(IntakeState desiredState) {
        switch (desiredState) {
            case IDLE:
                return this.runOnce(() -> this.stopIntake());

            case INTAKE:
                return this.runOnce(() -> this.intake());

            case OUTTAKE:
                return this.runOnce(() -> this.reverseIntake());

            default:
                return Commands.print("Invalid Intake State Provided!");

        }
    }

}