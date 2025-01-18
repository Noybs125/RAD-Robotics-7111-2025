package frc.robot.utils.encoder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

public class RevEncoder implements Encoder {
    private DutyCycleEncoder encoder;
    private double gearRatio = 1;
    private double position = 0;
    private double offset = 0;
    private boolean inverted = false;

    public RevEncoder(int id){
        encoder = new DutyCycleEncoder(id);
    }

    public Rotation2d getPosition(){
        return Rotation2d.fromRotations(position);
    }

    public void setPosition(Rotation2d position){
        offset = position.getRotations();
    }

    public boolean isAbsolute(){
        return false;
    }

    public double getGearRatio(){
        return gearRatio;
    }

    public void setGearRatio(double gearRatio){
        this.gearRatio = gearRatio;
    }

    public boolean getDirection(){
        return inverted;
    }

    public void periodic(){
        position = encoder.get() * gearRatio - offset;
        encoder.setInverted(inverted);
    }
}
