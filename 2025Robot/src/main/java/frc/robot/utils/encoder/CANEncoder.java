package frc.robot.utils.encoder;

import com.ctre.phoenix6.hardware.CANcoder;

import edu.wpi.first.math.geometry.Rotation2d;

public class CANEncoder implements Encoder {
    private final CANcoder encoder;

    public CANEncoder(int id) {
        encoder = new CANcoder(id);
    }

    public Rotation2d getPos(){
        return Rotation2d.fromRotations(encoder.getAbsolutePosition().getValueAsDouble());
    }

    public void setPos(Rotation2d postion){
        encoder.setPosition(postion);
    }

    public boolean isAbsolute(){}

    public double getVelocity(){}

    public double getVoltage(){}

    public void setGearRatio(){}
    
}
