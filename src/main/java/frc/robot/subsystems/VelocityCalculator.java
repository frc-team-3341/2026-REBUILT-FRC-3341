package frc.robot.subsystems;
public class VelocityCalculator {
    public double getRPM4Distance(double distance){
        double Circumference = 12.5;
        if(distance < 0.7){
            return 0.0;
        }
        double linVelocity = 1.4101*distance + 5.3292;
        double RPM = linVelocity/Circumference * 60;
        return RPM;
    }
}
