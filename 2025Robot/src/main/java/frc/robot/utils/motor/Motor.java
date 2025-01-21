package frc.robot.utils.motor;

import frc.robot.utils.encoder.Encoder;

public interface Motor {

    public void setSpeed(double speed);

    public double getSpeed();

    public void setPosition(double position);
    
    public double getPosition();
    
    public void setSetpoint(double setPoint);

    /** Must be called by the subystems periodic method */
    public void periodic();
    
    public void setPID(double p, double i, double d);
    public double getP();
    public double getI();
    public double getD();

    public Encoder getEncoder();

    public void setGearRatio(double gearRatio);

    public double getGearRatio();

    public double getVoltage();

    public boolean isAtSetpoint(double deadzone);

    public double getFeedFoward();

    public void setFeedFoward(double kS, double kV, double kA);
    
}
