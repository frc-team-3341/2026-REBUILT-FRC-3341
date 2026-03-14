
package frc.robot.commands;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.Command;

public class ClimberServoCommand extends Command{
    public static PWM pwm;
    public ClimberServoCommand() {
        pwm = new PWM(3);
        pwm.setBoundsMicroseconds(2500,0,1500,0,500);
        pwm.setPeriodMultiplier(PWM.PeriodMultiplier.k1X);
        pwm.setPosition(0);
    }
    public void setPulseTimeMicroseconds(int time) {
        pwm.setPulseTimeMicroseconds(time);
    }
    public void setAngle(double angle) {
        if(angle == 90.0) {
            pwm.setPosition(0.32);
        }
        else if(angle == 0.0) {
            pwm.setPosition(0);
        }
    }
}
