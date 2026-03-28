package frc.robot.commands;

import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootAuto extends Command{
    ShooterSubsystem shooter;
    Timer timer = new Timer();


    public ShootAuto(ShooterSubsystem shooter) {
        this.shooter = shooter;
    }

    @Override
    public void execute() {
        timer.start();
        shooter.score();

        if (shooter.readyToShoot()) {
            SmartDashboard.putNumber("encoder rpm", shooter.getShooterRPM());
            shooter.feed();
        }
        else {
            shooter.stopFeed();
        }
    }

    public boolean isFinished() {
        return timer.get() > 10;
    }
}
