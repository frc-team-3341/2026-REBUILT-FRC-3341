// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

// Rev Robotics imports
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {

  // initialize elements
  private SparkMax shooterWheel;
  private SparkClosedLoopController shooterWheelController;
  private RelativeEncoder shooterEncoder;

  // Constants
  private final int shooterWheelCanId = 4;

  // Variables
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM;
  public int setPoint; // In RPM

  public ShooterSubsystem() {

    // Initialize motor
    shooterWheel = new CANSparkMax(shooterWheelCanId, MotorType.kBrushless);
    shooterWheel.restoreFactoryDefaults();

    // Initialize PID controller and encoder
    shooterWheelController = shooterWheel.getPIDController();
    shooterEncoder = shooterWheel.getEncoder();

    // PID Constants
    kP = 6e-5; 
    kI = 0;
    kD = 0; 
    kIz = 0; 
    kFF = 0.000015; 
    kMaxOutput = 1; 
    kMinOutput = -1;
    maxRPM = 5700;

    // set pid values
    m_pidController.setP(kP);
    m_pidController.setI(kI);
    m_pidController.setD(kD);
    m_pidController.setIZone(kIz);
    m_pidController.setFF(kFF);
    m_pidController.setOutputRange(kMinOutput, kMaxOutput);

    // Shuffleboard
    private ShuffleboardTab tab = Shuffleboard.getTab("Shooter")

    tab.putNumber("P Gain", kP);
    tab.putNumber("I Gain", kI);
    tab.putNumber("D Gain", kD);
    tab.putNumber("I Zone", kIz);
    tab.putNumber("Feed Forward", kFF);
    tab.putNumber("Max Output", kMaxOutput);
    tab.putNumber("Min Output", kMinOutput);
    tab.putNumber("Set Point", setPoint);


  }
  
  public double getRPM4Distance(double distance){
        double Circumference = 12.5;
        if(distance < 0.7){
            return 0.0;
        }
        double linVelocity = 1.4101*distance + 5.3292;
        double RPM = linVelocity/Circumference * 60;
        return RPM;
    }
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {

    double p = tab.getNumber("P Gain", 0);
    double i = tab.getNumber("I Gain", 0);
    double d = tab.getNumber("D Gain", 0);
    double iz = tab.getNumber("I Zone", 0);
    double ff = tab.getNumber("Feed Forward", 0);
    double max = tab.getNumber("Max Output", 0);
    double min = tab.getNumber("Min Output", 0);

    // if PID coefficients on shuffleBoard have changed, write new values to controller
    if((p != kP)) { m_pidController.setP(p); kP = p; }
    if((i != kI)) { m_pidController.setI(i); kI = i; }
    if((d != kD)) { m_pidController.setD(d); kD = d; }
    if((iz != kIz)) { m_pidController.setIZone(iz); kIz = iz; }
    if((ff != kFF)) { m_pidController.setFF(ff); kFF = ff; }
    if((max != kMaxOutput) || (min != kMinOutput)) { 
      m_pidController.setOutputRange(min, max); 
      kMinOutput = min; kMaxOutput = max; 
    }

    setPoint = 3000;
    shooterWheelController.setReference(setPoint, CANSparkMax.ControlType.kVelocity);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
