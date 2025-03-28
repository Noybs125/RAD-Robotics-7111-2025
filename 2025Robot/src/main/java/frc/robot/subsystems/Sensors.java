package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Sensors extends SubsystemBase {

    public PowerDistribution powerHub = new PowerDistribution(1, ModuleType.kRev);
    private DigitalInput beamBreakRec = new DigitalInput(8);

    public Sensors(){
        Shuffleboard.getTab("DeviceOutputs").addDouble("PDHTotalCurrent", () -> getPDHCurrentTotal()).withWidget("");
        Shuffleboard.getTab("DeviceOutputs").addDouble("PDHBusVoltage", () -> getPDHVoltage()).withWidget("");
    }

    /**
     * Gets the current power distribution hub's input voltage.
     * Uses getVoltage method.
     * @return -Voltage from the power distribution hub in volts.
     * @see -Link to getVoltage method: https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/wpilibj/PowerDistribution.html#getVoltage().
     */
    public double getPDHVoltage(){
        double voltage = powerHub.getVoltage();
        return voltage;
    }

    /**
     * Gets the current power distribution hub's total current draw.
     * Uses getTotalCurrent method.
     * @return -Total current from power distribution hub in amperes, based on all channels.
     * @see -Link to getTotalCurrent method: https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/wpilibj/PowerDistribution.html#getTotalCurrent()
     */
    public double getPDHCurrentTotal(){
        double current = powerHub.getTotalCurrent();
        return current;
    }
    
    /**
     * Gets the current power draw of a specified channel.
     * Uses getCurrent(int) method.
     * @param channel -Type "int", specifies which channel to read from. Range between 0 and 23.
     * @return -Channel current in amperes. If givin invaled input, returns -1.
     * @see -Link to getCurrent(int) method: https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/wpilibj/PowerDistribution.html#getCurrent(int).
     */
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

    /**
     * Reads from digital pin to determine if the beambreak sensor on the end effector has been triggered.
     * Uses get method.
     * @return -The state of the beam break sensor pin, being on or off.
     * @see -Link to get method: https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/wpilibj/DigitalInput.html#get().
     */
    public boolean isBeamBroken(){
        return !beamBreakRec.get();
    }


    /**
     * Periodic method called 50 times per second. Currently completely empty.
     */
    public void periodic(){
        SmartDashboard.putBoolean("BeamBreakRec", isBeamBroken());
    }
}
