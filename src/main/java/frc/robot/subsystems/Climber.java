package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.LimitSwitchConfig.Behavior;
import com.revrobotics.spark.config.LimitSwitchConfig.Type;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;

import edu.wpi.first.wpilibj.DigitalInput;
import com.revrobotics.spark.SparkLimitSwitch;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.ClimberServoCommand;
import frc.robot.commands.DistanceSensorAlignmentCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
// import frc.robot.subsystems.DriveSubsystem;

public class Climber extends SubsystemBase {

    private final SparkFlex leadscrewMotor = new SparkFlex(13, MotorType.kBrushless); // need to check new CAN id's for the new robot
    private final RelativeEncoder encoder;
    private final SparkLimitSwitch topLimitSwitch; // need to check the channels for the limit switches
    private final SparkLimitSwitch bottomLimitSwitch;
    private final ClimberServoCommand servo;

    private static final double SERVO_ENGAGED_ANGLE = 90.0;
    private static final double SERVO_RELEASED_ANGLE = 0.0;
    private static final double SPEED = 0.2;
    private static final double SOFT_LIMIT_TOP = 95.0; // need to test the values for soft limits top and bottom
    private static final double SOFT_LIMIT_BOTTOM = 2.0; 

    public DistanceSensorAlignmentCommand rightSensor = new DistanceSensorAlignmentCommand(9);
    public DistanceSensorAlignmentCommand middleSensor = new DistanceSensorAlignmentCommand(2);
    public DistanceSensorAlignmentCommand leftSensor = new DistanceSensorAlignmentCommand(7);
    

    public Climber() {
        servo = new ClimberServoCommand();
        servo.setAngle(0);

        SparkFlexConfig config = new SparkFlexConfig();
        config.idleMode(IdleMode.kBrake);
        // config.softLimit
            // .forwardSoftLimit(SOFT_LIMIT_TOP)
            // .forwardSoftLimitEnabled(true)
            // .reverseSoftLimit(SOFT_LIMIT_BOTTOM)
            // .reverseSoftLimitEnabled(true);
        config.limitSwitch
            .forwardLimitSwitchType(Type.kNormallyClosed)
            .reverseLimitSwitchType(Type.kNormallyClosed);

        leadscrewMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        encoder = leadscrewMotor.getEncoder();
        encoder.setPosition(0);

        topLimitSwitch = leadscrewMotor.getForwardLimitSwitch();
        bottomLimitSwitch = leadscrewMotor.getReverseLimitSwitch();

    }

    // Leadscrew movement
    public Command leadscrewUp() {
        return this.runOnce(() -> {
            leadscrewMotor.set(SPEED);
        });
    }

    public Command leadscrewDown() {
        return this.runOnce(() -> {
            leadscrewMotor.set(-SPEED);
            
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
    public Command home() {
        return this.run(() -> {
            if(encoder.getPosition() < SOFT_LIMIT_TOP && !topLimitSwitch.isPressed()) {
                leadscrewMotor.set(SPEED);
            }
            else {
                leadscrewMotor.set(0);
            }

        })
        .until(() -> encoder.getPosition()>=SOFT_LIMIT_TOP || topLimitSwitch.isPressed())
        .andThen(this.runOnce(() -> {
            leadscrewMotor.set(0);
            encoder.setPosition(SOFT_LIMIT_TOP);
            servo.setAngle(SERVO_ENGAGED_ANGLE);

        }));
    }
    public Command autonomousAscend() {
        return this.runOnce(() -> servo.setAngle(SERVO_ENGAGED_ANGLE)).andThen(new WaitCommand(0.5)).andThen(this.run(() -> leadscrewMotor.set(SPEED)).until(() -> encoder.getPosition() >= SOFT_LIMIT_TOP || topLimitSwitch.isPressed())).andThen(leadscrewStop());
    }
    public Command autonomousDescend() {
        return this.run(() -> leadscrewMotor.set(-SPEED)).until(() -> encoder.getPosition() <=SOFT_LIMIT_BOTTOM || bottomLimitSwitch.isPressed()).andThen(leadscrewStop());
    }
    // Encoder utilities
    public double getPosition() {
        return encoder.getPosition();
    }

    public void resetEncoder() {
        encoder.setPosition(0);
    }

    // public Command alignToTower(DriveSubsystem drive) {
    //     Timer searchTimer = new Timer();
    //     int negOneMiddleCounter =0;
    //     int negOneRightCounter = 0;
    //     int negOneLeftCounter = 0;
    //     return this.run(() -> {
    //         boolean middle = middleSensor.getDistanceInches() >=0 && middleSensor.getDistanceInches() <=15;
    //         boolean left = leftSensor.getDistanceInches() >= 0 && leftSensor.getDistanceInches() < 15;
    //         boolean right = rightSensor.getDistanceInches() >=0 && rightSensor.getDistanceInches() <15;
    //         if(middle == false) {
    //             negOneMiddleCounter +=1;
    //         }
    //         if(left == false) {
    //             negOneLeftCounter +=1;
    //         }
    //         if(right == false) {
    //             negOneRightCounter+=1;
    //         }


    //         if(middle == true) {
    //             drive.drive(new ChassisSpeeds(0,0,0), false);
    //             negOneMiddleCounter = 0;
    //         }
    //         else if (right == true) {
    //             searchTimer.stop();
    //             searchTimer.reset();
    //             drive.drive(new ChassisSpeeds(0,0.3,0), false);
    //             negOneLeftCounter = 0;
    //         }
    //         else if(left == true) {
    //             searchTimer.stop();
    //             searchTimer.reset();
    //             drive.drive(new ChassisSpeeds(0,-0.3,0), false);
    //             negOneRightCounter = 0;
    //         }
    //         else if (negOneMiddleCounter>25 && negOneLeftCounter >25 && negOneRightCounter >25){
    //             if(!searchTimer.isRunning()) {
    //                 searchTimer.start();
    //             }
    //             double tim = searchTimer.get();
    //             if(tim < 1) {
    //                 drive.drive(new ChassisSpeeds(0,-0.3, 0), false);
    //             }
    //             else if(tim < 2) {
    //                 drive.drive(new ChassisSpeeds(0,0.3,0), false);
    //             }
    //             else if(tim<3) {
    //                 drive.drive(new ChassisSpeeds(0,0.3,0), false);
    //             }
    //             else {
    //                 drive.drive(new ChassisSpeeds(0,0,0), false);
    //             }

    //         }

    //     }) 
    //     .until(() -> middleSensor.getDistanceInches() >=0 && middleSensor.getDistanceInches() < 10)
    //     .finallyDo(() -> drive.drive(new ChassisSpeeds(0,0,0), false));
    // }

    // public Command moveForward(DriveSubsystem drive) {
    //     return this.run(() -> {
    //         drive.drive(new ChassisSpeeds(0.3, 0,0), false);
    //     })
    //     .until(() -> middleSensor.getDistanceMM() <= 7.9375)
    //     .finallyDo(() -> {
    //         drive.drive(new ChassisSpeeds(0,0,0), false);
    //     });
    // }
//AMOGH WAS COOKING HERE
    @Override
    public void periodic() {
        // Hard stop using encoder and limit switches
        if ((encoder.getPosition() >= SOFT_LIMIT_TOP || topLimitSwitch.isPressed()) && leadscrewMotor.get() > 0) {
            leadscrewMotor.set(0);
        }
        if ((encoder.getPosition() <= SOFT_LIMIT_BOTTOM || bottomLimitSwitch.isPressed()) && leadscrewMotor.get() < 0) {
            leadscrewMotor.set(0);
        }
        
        SmartDashboard.putNumber("Position", encoder.getPosition());
        SmartDashboard.putBoolean("TopLimitSwitch", topLimitSwitch.isPressed());
        SmartDashboard.putBoolean("BottomLimitSwitch", bottomLimitSwitch.isPressed());
        
    }
}