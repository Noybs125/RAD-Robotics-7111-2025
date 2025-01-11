package frc.robot.utils.motor;

public interface Motor {

    public void setSpeed(double speed);


    public double getSpeed();
    

    public void setPosition(double position);

    
    public double getPosition();
        
    
    public void setSetpoint(double setPoint);

    
    public void periodic();

    
    
}
