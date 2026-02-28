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
    private SparkFlex frontWheel;
    private SparkFlexConfig frontConfig;

    private DigitalInput beamBreak;
    private DigitalInput beamBreak1;
    private int counter;
    private boolean motorIsOn;

    public Intake(){
        //The motor we're using for the kitbot is kBrushed
        intakeWheel = new SparkFlex(Constants.DriveConstants.intakeMotorCanId, MotorType.kBrushless);
        intakeConfig = new SparkFlexConfig();

        //This isn't the actually CAN ID being used. It is just a placeholder until we get another motor
        frontWheel = new SparkFlex(9, MotorType.kBrushless);
        frontConfig = new SparkFlexConfig();
        
        motorIsOn = false;
        //This is the beambreak sensor that senses whether the beam is hitting it or not
        //beamBreak = new DigitalInput(3);
        counter = 0;
        
        //The channel is just a placeholder as we only have one beam break sensor currently
        //beamBreak1 = new DigitalInput(2);

        intakeConfig.closedLoop.pid(0.01, 0, 0);
        frontConfig.closedLoop.pid(0.01, 0, 0);

        // intakeWheel.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        intakeWheel.configure(intakeConfig, com.revrobotics.ResetMode.kResetSafeParameters, com.revrobotics.PersistMode.kPersistParameters);
        frontWheel.configure(frontConfig, com.revrobotics.ResetMode.kResetSafeParameters, com.revrobotics.PersistMode.kPersistParameters  );

    }

    // public boolean getBeamBreak(){
    //     return beamBreak.get();
    // }

    // public boolean getBallPassedThrough(){
    //     return !beamBreak.get();
    // }

    @Override
    public void periodic() {
        //counter++;0o++++++++++++++++ ard.putBoolean("beambreak state", getBeamBreak());
        // System.out.println(motorIsOn);
        //SmartDashboard.putBoolean("Ball Passed Through", getBallPassedThrough());
    }
    public int getCounter(){
        return counter;
    }
    public void addCounter(int x){
        System.out.println(getCounter());
        counter += x;
    }
    public void setCounter(int x){
        counter = x;
    }
    public void setMotorOn(boolean val){
        motorIsOn = val; 
    }
    public boolean getMotorOn(){
        return motorIsOn;
    }

    public Command setMotorCommandOn(boolean val){
        return runOnce(
            () -> {
                motorIsOn = val; 
        });
    }

    public Command intakeBall() {
        return runOnce(
            () -> {
                intakeWheel.set(0.25);
                frontWheel.set(-0.15);
        });
    }

    public Command reverseIntakeBall() {
        return runOnce(
            () -> {
                intakeWheel.set(-0.25);
                frontWheel.set(0.15);
        });
    }
    
    public Command stopIntake(){
        return runOnce(
            () -> {
                intakeWheel.set(0);
                frontWheel.set(0);
        });
    }


     public Command keepOn(){
        return runOnce(() -> {
            if(getMotorOn()){
                setMotorOn(false);
            }
            else{
             setMotorOn(true);
            }
        });
    }

}
