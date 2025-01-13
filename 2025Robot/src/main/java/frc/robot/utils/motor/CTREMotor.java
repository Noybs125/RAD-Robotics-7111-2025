package frc.robot.utils.motor;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.PositionVoltage;
import edu.wpi.first.math.controller.PIDController;

public class CTREMotor implements Motor {
    private TalonFX motor;
    PositionVoltage setpos = new PositionVoltage(0);
    PIDController pid = new PIDController(0, 0, 0);
    
    public CTREMotor(int id){
        motor = new TalonFX(id);

    }

    public void setSpeed(double speed){
        motor.set(speed);
    }


    public double getSpeed(){
        return motor.get();
    }
    

    public void setPosition(double position){
        motor.setPosition(position);
    }

    
    public double getPosition(){
        return motor.getPosition().getValueAsDouble();
    }
        
    
    public void setSetpoint(double setPoint){
        pid.setSetpoint(setPoint);
        setpos.Position = setPoint;
        motor.setControl(setpos);
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
