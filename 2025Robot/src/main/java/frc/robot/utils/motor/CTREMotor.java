package frc.robot.utils.motor;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.PositionVoltage;

public class CTREMotor implements Motor {

    TalonFX motor = new TalonFX(0);
    PositionVoltage setpos = new PositionVoltage(0);

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
        setpos.Position = setPoint;
        motor.setControl(setpos);
    }

    
    public void periodic(){

    }

    
    
}
