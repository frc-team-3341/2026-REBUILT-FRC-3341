package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {

    private final SparkFlex leadscrewMotor = new SparkFlex(14, MotorType.kBrushless); // need to check new CAN id's for the new robot
    private final RelativeEncoder encoder;
    private final DigitalInput topLimit    = new DigitalInput(0); // need to check the channels for the limit switches
    private final DigitalInput bottomLimit = new DigitalInput(1);
    private final Servo servo = new Servo(0);

    private static final double SERVO_ENGAGED_ANGLE  = 90.0;
    private static final double SERVO_RELEASED_ANGLE = 0.0;
    private static final double SPEED = 0.2;
    private static final double SOFT_LIMIT_TOP    = 95.0; // need to test the values for soft limits top and bottom
    private static final double SOFT_LIMIT_BOTTOM = 2.0; 

    public Climber() {
    SparkFlexConfig config = new SparkFlexConfig();
    config.idleMode(IdleMode.kBrake);
    config.softLimit
    
    // tells the motor to break if it gets to top or bottom limits

        .forwardSoftLimit(SOFT_LIMIT_TOP)
        .forwardSoftLimitEnabled(true)
        .reverseSoftLimit(SOFT_LIMIT_BOTTOM)
        .reverseSoftLimitEnabled(true);

    leadscrewMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    encoder = leadscrewMotor.getEncoder();
    encoder.setPosition(0);
}

    // Leadscrew movement
    public Command leadscrewUp() {
        return this.runOnce(() -> {
            if (encoder.getPosition() < SOFT_LIMIT_TOP && !topLimit.get()) {
                leadscrewMotor.set(SPEED);
            }
        });
    }

    public Command leadscrewDown() {
        return this.runOnce(() -> {
            if (encoder.getPosition() > SOFT_LIMIT_BOTTOM && !bottomLimit.get()) {
                leadscrewMotor.set(-SPEED);
            }
        });
    }

    public Command leadscrewStop() {
        return this.runOnce(() -> leadscrewMotor.set(0.0));
    }

    // Servo movement
    public Command servoOn() {
        return this.runOnce(() -> servo.setAngle(SERVO_ENGAGED_ANGLE));
    }

    public Command servoOff() {
        return this.runOnce(() -> servo.setAngle(SERVO_RELEASED_ANGLE));
    }

    // Encoder utilities
    public double getPosition() {
        return encoder.getPosition();
    }

    public void resetEncoder() {
        encoder.setPosition(0);
    }

    @Override
    public void periodic() {
        // Hard stop using encoder and limit switches
        if ((encoder.getPosition() >= SOFT_LIMIT_TOP || topLimit.get()) 
                && leadscrewMotor.get() > 0) {
            leadscrewMotor.set(0);
        }
        if ((encoder.getPosition() <= SOFT_LIMIT_BOTTOM || bottomLimit.get()) 
                && leadscrewMotor.get() < 0) {
            leadscrewMotor.set(0);
        }
        
        SmartDashboard.putNumber("Position", encoder.getPosition());
    }
}