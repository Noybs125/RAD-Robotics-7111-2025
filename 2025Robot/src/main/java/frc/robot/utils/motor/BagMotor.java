package frc.robot.utils.motor;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.math.controller.PIDController;

public class BagMotor implements Motor{
    private VictorSPX motor; 
    private PIDController pid = new PIDController(0,0,0);

    public BagMotor(int id){
        motor = new VictorSPX(id);
    }

    public void setSpeed(double speed){
        motor.set(VictorSPXControlMode.PercentOutput, speed);
    }

    public double getSpeed(){
        return motor.getMotorOutputPercent();
    }

    public void setPosition(double position){
    }
    
    public double getPosition(){
        return 0;
    }        
    
    public void setSetpoint(double setPoint){
        motor.set(VictorSPXControlMode.PercentOutput, pid.calculate(getPosition(), setPoint));
    }
    
    public void periodic(){

    }

    public void setPID(double P, double I, double D){
        pid.setPID(P, I, D);
    }

    public double getP(){
        return pid.getP();
    }

    public double getI(){
        return pid.getI();
    }

    public double getD(){
        return pid.getD();
    }

}
