package frc.robot.utils.motor;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import frc.robot.utils.encoder.Encoder;

public class TwoMotors implements Motor {

    private Motor motor1;
    private Motor motor2;

    public TwoMotors(Motor motor1, Motor motor2){
        this.motor1 = motor1;
        this.motor2 = motor2;
    }
    
    public void setSpeed(double speed){
        motor1.setSpeed(speed);
        motor2.setSpeed(speed);
    }

    public double getSpeed(){
        return motor1.getSpeed();
    }

    public void setPosition(double position){
        motor1.setPosition(position);
        motor2.setPosition(position);
    }
    
    public double getPosition(){
        return motor1.getPosition();
    }
    
    public void setSetpoint(double setPoint){
        motor1.setSetpoint(setPoint);
        motor2.setSetpoint(setPoint);
    }

    /** Must be called by the subystems periodic method */
    public void periodic(){
        motor1.periodic();
        motor2.periodic();
    }
    
    public void setPID(double p, double i, double d){
        motor1.setPID(p, i, d);
        motor2.setPID(p, i, d);
    }
    
    public PIDController getPID(){
        return motor1.getPID();
    }

    public Encoder getEncoder(){
        return motor1.getEncoder();
    }

    public double getVoltage(){
        return motor1.getVoltage();
    }

    public boolean isAtSetpoint(double deadzone){
        return motor1.isAtSetpoint(deadzone);
    }

    public SimpleMotorFeedforward getFeedForward(){
        return motor1.getFeedForward();
    }

    public void setFeedFoward(double kS, double kV, double kA){
        motor1.setFeedFoward(kS, kV, kA);
        motor2.setFeedFoward(kS, kV, kA);
    }
}
