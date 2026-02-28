package frc.robot.subsystems.Shooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import frc.util.FuelSim;

public class ShooterIOSim implements ShooterIO{
    
    FuelSim fuelSim;
    
    double expectedLinearVelocity;

    double flywheelRadius=0.1016;

    boolean feederOn;

    public ShooterIOSim(FuelSim fuelSim){
        this.fuelSim = fuelSim;
    }
    
    public void updateInputs(ShooterIOInputs inputs) {

    }

    public void setFlywheelRPM(double RPM) {
        expectedLinearVelocity=flywheelRadius*RPM/120;
        if (feederOn){
            fuelSim.launchFuel(MetersPerSecond.of(expectedLinearVelocity), Degrees.of(75), Degrees.of(0), Meters.of(0.7450328));
            try {
                Thread.sleep(500);
            }
            catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }

    public void setFeederSpeed(double speed) {
        if (speed>0){
            feederOn=true;
        }
        else{
            feederOn=false;
        }
    }


}
