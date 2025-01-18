package frc.robot.utils.motor;

import frc.robot.utils.encoder.Encoder;

public interface Motor {

    public void setSpeed(double speed);


    public double getSpeed();
    

    public void setPosition(double position);

    
    public double getPosition();
        
    
    public void setSetpoint(double setPoint);

    
    public void periodic();

    
    public void setPID(double P, double I, double D);
    public double getP();
    public double getI();
    public double getD();

    public Encoder getEncoder();
    
}
