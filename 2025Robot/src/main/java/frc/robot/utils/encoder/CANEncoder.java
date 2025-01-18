package frc.robot.utils.encoder;

import com.ctre.phoenix6.hardware.CANcoder;

import edu.wpi.first.math.geometry.Rotation2d;

public class CANEncoder implements Encoder {
    private final CANcoder encoder;
    private double gearRatio = 1;

    public CANEncoder(int id) {
        encoder = new CANcoder(id);
    }

    public CANEncoder(int id, double gearRatio) {
        this.gearRatio = gearRatio;
        encoder = new CANcoder(id);
    }

    public Rotation2d getPosition(){
        return Rotation2d.fromRotations(encoder.getAbsolutePosition().getValueAsDouble() * gearRatio);
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

    public void setGearRatio(double gearRatio){
        this.gearRatio = gearRatio;
    }
    
}
