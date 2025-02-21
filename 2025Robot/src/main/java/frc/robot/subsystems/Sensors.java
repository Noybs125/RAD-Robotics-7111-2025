package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Sensors extends SubsystemBase {

    public PowerDistribution powerHub = new PowerDistribution(1, ModuleType.kRev);
    private DigitalInput beamBreak = new DigitalInput(2);

    public Sensors(){
        Shuffleboard.getTab("DeviceOutputs").addDouble("PDHPower", () -> getPDHPower()).withWidget("");
        Shuffleboard.getTab("DeviceOutputs").addDouble("PDHEnergy", () -> getPDHEnergy()).withWidget("");
        Shuffleboard.getTab("DeviceOutputs").addDouble("PDHTotalCurrent", () -> getPDHCurrentTotal()).withWidget("");
        Shuffleboard.getTab("DeviceOutputs").addDouble("PDHTemp", () -> getPDHTemp()).withWidget("");
        Shuffleboard.getTab("DeviceOutputs").addDouble("PDHBusVoltage", () -> getPDHVoltage()).withWidget("");
    }

    //Power Distribution Sensors
    //get bus voltage
    public double getPDHVoltage(){
        double voltage = powerHub.getVoltage();
        return voltage;
    }

    //gets total PDH current
    public double getPDHCurrentTotal(){
        double current = powerHub.getTotalCurrent();
        return current;
    }
    //gets specified channel current
    public double getPDHChannelCurrent(int channel){
        if(channel <= 23 && channel >= 0){
        double channelCurrent = powerHub.getCurrent(channel);
        return channelCurrent;
        }
        else{
            //invalid channel check
            return -1;
        }
    }

    //gets PDH temperature
    public double getPDHTemp(){
        double temp = (powerHub.getTemperature() * 9/5) + 32;
        return temp;
    }

    //gets PDH total energy and power
    public double getPDHEnergy(){
        double totalEnergy = powerHub.getTotalEnergy();
        return totalEnergy;
    }

    public double getPDHPower(){
        double totalPower = powerHub.getTotalPower();
        return totalPower;
    }

    public boolean isBeamBroken(){
        return beamBreak.get();
    }


    public void periodic(){}
}
