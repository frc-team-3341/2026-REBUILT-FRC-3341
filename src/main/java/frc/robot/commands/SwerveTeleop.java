package frc.robot.commands;

import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Drive;
import frc.util.JoystickUtil;

public class SwerveTeleop extends Command{

    CommandXboxController cont;

    SlewRateLimiter translationRateLimiter = new SlewRateLimiter(5, 1000, 0);
    SlewRateLimiter rotationalRateLimiter = new SlewRateLimiter(10, 10, 0);
    
    Drive swerve;
    
    double xInput;
    double yInput;

    double rotInput;

    BooleanSupplier aimDriveSupplier;

    public SwerveTeleop(Drive swerve, CommandXboxController cont, BooleanSupplier aimDriveSupplier) {
        this.swerve = swerve;
        this.cont = cont;
        this.aimDriveSupplier = aimDriveSupplier;

        addRequirements(swerve);
    }


    @Override
    public void execute() {

        Logger.recordOutput("aim drive enabled", aimDriveSupplier.getAsBoolean());

        xInput = -cont.getLeftY();
        yInput = -cont.getLeftX();
        rotInput = -cont.getRightX();

        xInput = MathUtil.applyDeadband(xInput, Constants.OIConstants.kDriveDeadband);
        yInput = MathUtil.applyDeadband(yInput, Constants.OIConstants.kDriveDeadband);
        rotInput = MathUtil.applyDeadband(rotInput, Constants.OIConstants.kDriveDeadband);

        double[] vals = 
            JoystickUtil.scaleJoystickInputs(xInput, yInput, 
                DriveConstants.kMaxSpeedMetersPerSecond);

        //New scaled values for x and y
        xInput = vals[1]*Math.cos(vals[0]);
        yInput = vals[1]*Math.sin(vals[0]);

        rotInput = Math.copySign(rotInput*rotInput, rotInput);

        rotInput *= DriveConstants.kMaxAngularSpeed;

        if (aimDriveSupplier.getAsBoolean()) {
            swerve.aimDrive(xInput, yInput);
        }
        else {
            swerve.drive(new ChassisSpeeds(xInput, yInput, rotInput), true);
        }



    }

    @Override
    public void end(boolean interrupted) {
        swerve.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}