package frc.robot.commands;

import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;


public class DistanceSensorAlignmentCommand extends Command {
    private final Counter counter;
    private final DistanceSensorAlignmentCommand left = new DistanceSensorAlignmentCommand(0);
    private final DistanceSensorAlignmentCommand right  = new DistanceSensorAlignmentCommand(1);
    private final DistanceSensorAlignmentCommand middle = new DistanceSensorAlignmentCommand(2);
    //private double distanceMillimeters;

    
    public DistanceSensorAlignmentCommand(int dioPort) {
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
    
}