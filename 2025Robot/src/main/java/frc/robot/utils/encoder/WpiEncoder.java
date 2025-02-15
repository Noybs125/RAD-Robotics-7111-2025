package frc.robot.utils.encoder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import frc.robot.Constants;

public class WpiEncoder implements frc.robot.utils.encoder.Encoder {
    private Encoder encoder;
    private double gearRatio = 1;
    private double position = 0;
    private double offset = 0;
    private boolean inverted = false;
    private int id;

    public WpiEncoder(int id, int id2){
        encoder = new Encoder(id,id2);
        EncoderSim encoderSim = new EncoderSim(encoder);
        encoder.setDistancePerPulse(Constants.kSimulation.elevatorEncoderDistPerPulse);
        this.id = id;
    }

    public WpiEncoder(int id, int id2, double gearRatio){
        encoder = new Encoder(id, id2);
        EncoderSim encoderSim = new EncoderSim(encoder);
        encoder.setDistancePerPulse(Constants.kSimulation.elevatorEncoderDistPerPulse);
        this.id = id;
        this.gearRatio = gearRatio;
    }

    public Rotation2d getPosition(){
        return Rotation2d.fromRotations(position);
    }

    public double getPositionAsDouble(){
        return position;
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

    public void setDirection(boolean inverted){
        this.inverted = inverted;
    }

    public void periodic(){
        position = encoder.get() * gearRatio - offset;
        encoder.setReverseDirection(inverted);

        //Shuffleboard stuff goes here
    }
}
