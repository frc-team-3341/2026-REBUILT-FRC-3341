package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class DistanceSensor extends SubsystemBase {
    private final Counter counter;
    private final DistanceSensor left = new DistanceSensor(0);
    private final DistanceSensor right  = new DistanceSensor(1);
    private final DistanceSensor middle = new DistanceSensor(2);
    //private double distanceMillimeters;

    
    public DistanceSensor(int dioPort) {
        DigitalInput input = new DigitalInput(dioPort);
        counter = new Counter(input);
        counter.setSemiPeriodMode(true);
    }

    public double getDistanceMM() {
        double pulseWidthMicros = counter.getPeriod() * 1e6;
        if (pulseWidthMicros > 1850) return -1;
        double distance = (pulseWidthMicros - 1000) * 3.0 / 4.0;
        return Math.max(0, distance);
    }

    public double getDistanceInches() {
        return getDistanceMM() / 25.4;
    }
    
    @Override
    public void periodic() {
    // This method will be called once per scheduler run
       
        SmartDashboard.putNumber("Inches Value (Left)", left.getDistanceInches());
        SmartDashboard.putNumber("Inches Value (Right)", right.getDistanceInches());
        SmartDashboard.putNumber("Inches Value (Middle)", middle.getDistanceInches());

    }
}