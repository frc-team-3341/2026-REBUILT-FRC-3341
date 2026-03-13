package frc.robot.subsystems;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.subsystems.Superstructure.IntakeState;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import static frc.robot.Constants.IntakeConstants.*;

import java.io.ObjectInputFilter.Config;

public class Intake extends SubsystemBase{
    private SparkFlex liftWheel;
    private SparkFlex intakeWheel;

    private DigitalInput beamBreak;
    private DigitalInput beamBreak1;

    public Intake(){
        //The motor we're using for the kitbot is kBrushed
        liftWheel = new SparkFlex(LIFT_CAN_ID, MotorType.kBrushless);

        //This isn't the actually CAN ID being used. It is just a placeholder until we get another motor
        intakeWheel = new SparkFlex(INTAKE_CAN_ID, MotorType.kBrushless);
        
        //This is the beambreak sensor that senses whether the beam is hitting it or not
        //beamBreak = new DigitalInput(3);
        
        //The channel is just a placeholder as we only have one beam break sensor currently
        //beamBreak1 = new DigitalInput(2);

        // intakeWheel.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        intakeWheel.configure(Configs.Intake.INTAKE_CONFIG, com.revrobotics.ResetMode.kResetSafeParameters, com.revrobotics.PersistMode.kPersistParameters  );
        liftWheel.configure(Configs.Intake.LIFT_CONFIG, com.revrobotics.ResetMode.kResetSafeParameters, com.revrobotics.PersistMode.kPersistParameters);

    }

    @Override
    public void periodic() {
    
    }
    //fix this to make sure intake wheel is inverted
    public void intake() {
        liftWheel.set(0.67);
        intakeWheel.set(-0.75);
    }

    //fix this to make sure intake wheel is inverted
    public void reverseIntake() {
        liftWheel.set(-0.67);
        intakeWheel.set(0.75);
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
