package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;

public class ClimbAuto extends Command{
    Climber climber;
    int counter;

    public ClimbAuto(Climber climber) {
        this.climber = climber;

        addRequirements(climber);
    }

    @Override
    public void initialize() {
        counter = 0;
    }

    @Override
    public void execute() {
        climber.climb();
        counter++;
    }

    @Override
    public void end(boolean interrupted) {
        climber.stop();
    }

    @Override
    public boolean isFinished() {
        return counter >= 250;
    }
}
