package frc.robot.utils.encoder;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

public class CANEncoder implements Encoder {
    private final CANcoder encoder;
    private double gearRatio = 1;
    private SensorDirectionValue isCW = SensorDirectionValue.Clockwise_Positive;

    public CANEncoder(int id) {
        encoder = new CANcoder(id);
    }

    public CANEncoder(int id, double gearRatio, SensorDirectionValue isCW) {
        this.gearRatio = gearRatio;
        encoder = new CANcoder(id);
        this.isCW = isCW;

        Shuffleboard.getTab("DeviceOutputs").add("CANEncoderVolt", getVoltage()).withWidget("").getEntry();
    }

    public Rotation2d getPosition(){
        return Rotation2d.fromRotations(encoder.getAbsolutePosition().getValueAsDouble() * gearRatio);
    }

    public double getPositionAsDouble(){
         return encoder.getAbsolutePosition().getValueAsDouble() * gearRatio;
    }

    public void setPosition(Rotation2d postion){
        encoder.setPosition(postion.getRotations());
    }

    public boolean isAbsolute(){
        return true;
    }

    public double getVelocity(){
        return encoder.getVelocity().getValueAsDouble();
    }

    public double getVoltage(){
        return encoder.getSupplyVoltage().getValueAsDouble();
    }

    public double getGearRatio() {
        return gearRatio;
    }

    public void setGearRatio(double gearRatio){
        this.gearRatio = gearRatio;
    }

    public void setDirection(boolean isCW){
        if (isCW)
            this.isCW = SensorDirectionValue.Clockwise_Positive;
        else   
            this.isCW = SensorDirectionValue.CounterClockwise_Positive;
    }

    public boolean getDirection(){
        if (isCW == SensorDirectionValue.Clockwise_Positive)
            return true;
        else 
            return false;
    }

    public void periodic(){

    }
    
}
