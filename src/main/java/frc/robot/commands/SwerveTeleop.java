package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.util.JoystickUtil;

public class SwerveTeleop extends Command{
    //TODO ADD ASYMMETERIC LIMITER PLEASE

    CommandXboxController cont;
    DriveSubsystem swerve;
    
    double xInput;
    double yInput;

    double rotInput;

    public SwerveTeleop(DriveSubsystem swerve, CommandXboxController cont) {
        this.swerve = swerve;
        this.cont = cont;

        addRequirements(swerve);
    }


    @Override
    public void execute() {
        xInput = -cont.getLeftY(); //Not a good practice, but this seems to be the way we've dont it with our old swerve code, looks to be a quirk of our controller mapping. Advantage scope sees the right direction of the modules, its just the controller thats werid.
        yInput = -cont.getLeftX();
        rotInput = cont.getRightX();

        xInput = MathUtil.applyDeadband(xInput, Constants.OIConstants.kDriveDeadband);
        yInput = MathUtil.applyDeadband(yInput, Constants.OIConstants.kDriveDeadband);
        rotInput = MathUtil.applyDeadband(rotInput, Constants.OIConstants.kDriveDeadband);

        double[] vals = 
            JoystickUtil.scaleJoystickInputs(xInput, yInput, 
                DriveConstants.kMaxSpeedMetersPerSecond);

        //New scaled values for x and y
        xInput = vals[1]*Math.cos(vals[0]);
        yInput = vals[1]*Math.sin(vals[0]);

        rotInput *= DriveConstants.kMaxAngularSpeed;

        swerve.drive(xInput, yInput, rotInput, true);



    }

    @Override
    public void end(boolean interrupted) {
        swerve.stopMotors();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
