package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.LimitSwitchConfig.Type;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;

import edu.wpi.first.wpilibj.PWM;

import com.revrobotics.spark.SparkLimitSwitch;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.ClimberConstants.*;

public class Climber extends SubsystemBase {

    private final SparkFlex leadscrewMotor = new SparkFlex(13, MotorType.kBrushless); 
    private final RelativeEncoder encoder;
    private final SparkLimitSwitch topLimitSwitch; // need to check the channels for the limit switches
    private final SparkLimitSwitch bottomLimitSwitch;
    private final PWM pwm;

    int pulseTime = 2000;

    public Climber() {
        pwm = new PWM(0);
        // servo.setAngle(0);

        SparkFlexConfig config = new SparkFlexConfig();
        config.idleMode(IdleMode.kCoast);
        // config.softLimit
            // .forwardSoftLimit(SOFT_LIMIT_TOP)
            // .forwardSoftLimitEnabled(true)
            // .reverseSoftLimit(SOFT_LIMIT_BOTTOM)
            // .reverseSoftLimitEnabled(true);
        config.limitSwitch
            .forwardLimitSwitchType(Type.kNormallyClosed)
            .reverseLimitSwitchType(Type.kNormallyClosed);
        
        //Needs to be inverted because the limit switches are inverted electrically
        config.inverted(true);

        leadscrewMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        encoder = leadscrewMotor.getEncoder();
        encoder.setPosition(0);

        topLimitSwitch = leadscrewMotor.getForwardLimitSwitch();
        bottomLimitSwitch = leadscrewMotor.getReverseLimitSwitch();

        SmartDashboard.putNumber("pulseTime", pulseTime);

    }

    // Leadscrew movement
    public Command leadscrewDown() {
        return this.runOnce(() -> {
            System.out.println("Going Down!");
            leadscrewMotor.set(SPEED);
        }).alongWith(deploy());
    }

    public Command leadscrewUp() {
        return this.runOnce(() -> {
            System.out.println("Going Up!");
            leadscrewMotor.set(-SPEED);
            
        }).alongWith(deploy());
    }

    public void climb() {
        leadscrewMotor.set(SPEED);
    }

    public void stop() {
        leadscrewMotor.set(0);
    }

    public Command leadscrewStop() {
        return this.runOnce(() -> leadscrewMotor.set(0.0));
    }

    // Encoder utilities
    public double getPosition() {
        return encoder.getPosition();
    }

    public void resetEncoder() {
        encoder.setPosition(0);
    }

    public Command setPulseTime() {
        return this.runOnce(() -> {
            System.out.println(pulseTime);
            pwm.setPulseTimeMicroseconds(pulseTime);
        });
    }

    public Command deploy() {
        return Commands.runOnce(() -> pwm.setPulseTimeMicroseconds(SERVO_ENGAGED_PULSE));
    }

    public Command retract() {
        return Commands.runOnce(() -> pwm.setPulseTimeMicroseconds(SERVO_RELEASED_PULSE));
    }

    @Override
    public void periodic() {
        
        SmartDashboard.putNumber("Position", encoder.getPosition());
        SmartDashboard.putBoolean("TopLimitSwitch", topLimitSwitch.isPressed());
        SmartDashboard.putBoolean("BottomLimitSwitch", bottomLimitSwitch.isPressed());

        pulseTime = (int) SmartDashboard.getNumber("pulseTime", pulseTime);
        
    }
}