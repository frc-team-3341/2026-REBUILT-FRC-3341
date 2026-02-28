package frc.robot.subsystems.Shooter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Superstructure.FeederState;
import frc.robot.subsystems.Superstructure.ShooterState;
import frc.util.ShooterUtil;
import static frc.robot.Constants.ShooterConstants.*;

public class Shooter extends SubsystemBase {
    private double lastPose = 0.0;
    private double flywheelRPM = 0.0;
    ShooterIO io;
    ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

    public Shooter(ShooterIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
    }

    public Command handleFeederTransition(FeederState desiredState) {
        switch (desiredState) {
            case IDLE:
                return Commands.runOnce(() -> io.stopFeeder());

            case FEED:
                return Commands.runOnce(() -> io.setFeederSpeed(FEEDING_SPEED));

            case BACKFEED:
                return Commands.runOnce(() -> io.reverseFeed(BACKFEED_SPEED));
                
            default:
                return Commands.print("Invalid Feeder State Provided!");

        }
    }

    //TODO finish ts
    public Command handleShooterTransitions(ShooterState desiredState) {
        switch (desiredState) {
            case IDLE:
                return this.runOnce(() -> io.stopFlywheel());

            case PASSING:
                return this.runOnce(() -> io.setFlywheelRPM(PASSING_RPM));

            //TODO reimplement this so that flywheelRPM isn't constantly recalculated, only if
            //there is a significant change in distance
            case SCORING:
                return this.run(() -> {
                    Pose2d robotPose = RobotContainer.getPose();
                
                    double distanceToHub = ShooterUtil.getDistanceToHub(robotPose);

                if (Math.abs(distanceToHub-lastPose)>0.05) {
                    System.out.println("UPDATING THE SPEED:" + distanceToHub);
                //    flywheelRPM = speedMap.get(distanceToHub);
                    lastPose = distanceToHub;
                }
                io.setFlywheelRPM(flywheelRPM);
                //before starting get distance to hub so not storing 0.0 in lastPose
                }).beforeStarting(() -> {
                    lastPose = ShooterUtil.getDistanceToHub(RobotContainer.getPose());
                    //get flywheel spinning to prevent delay
                    // flywheelRPM = speedMap.get(lastPose);
                });
                    
            default:
                return Commands.print("Invalid Shooter State Provided!");
        }
    }
}
