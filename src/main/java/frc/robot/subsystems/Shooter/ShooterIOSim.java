package frc.robot.subsystems.Shooter;

import frc.util.FuelSim;

public class ShooterIOSim implements ShooterIO{
    
    FuelSim fuelSim;


    public ShooterIOSim(FuelSim fuelSim){
        this.fuelSim = fuelSim;
    }
    
    public void updateInputs(ShooterIOInputs inputs) {

    }

    public void setFlywheelRPM(double RPM) {
        
    }

    public void setFeederSpeed(double speed) {

    }


}
