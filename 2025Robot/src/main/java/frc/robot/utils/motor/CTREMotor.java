package frc.robot.utils.motor;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix6.controls.PositionVoltage;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.utils.encoder.Encoder;

public class CTREMotor implements Motor {
    private TalonFX motor;
    PIDController pid = new PIDController(0, 0, 0);
    private Encoder encoder = null;
    private double gearRatio;
    
    public CTREMotor(int id){
        motor = new TalonFX(id);
    }

    public CTREMotor(int id, double gearRatio){
        motor = new TalonFX(id);
        this.gearRatio = gearRatio;
    }

    public CTREMotor(int id, Encoder encoder){
        this.encoder = encoder;

        motor = new TalonFX(id);
    }

    public CTREMotor(int id, Encoder encoder, double gearRatio){
        motor = new TalonFX(id);
        this.encoder = encoder;
        this.gearRatio = gearRatio;
    }

    public void setSpeed(double speed){
        motor.set(speed);
    }

    public double getSpeed(){
        return motor.get();
    }
    
    public void setPosition(double position){
        if(encoder != null){
            encoder.setPosition(Rotation2d.fromDegrees(position));
        } else {
            motor.setPosition(position / gearRatio);
        }
    }

    
    public double getPosition(){
        if(encoder == null){
            return motor.getPosition().getValueAsDouble() * gearRatio;
        } else{
            return encoder.getPosition().getDegrees();
        }
    }
        
    
    public void setSetpoint(double setPoint){
        motor.set(pid.calculate(getPosition(), setPoint));
    }

    public void periodic(){

    }

    public void setPID(double p, double i, double d){
        pid.setPID(p, i, d);
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

    public Encoder getEncoder(){
        return encoder;
    }    

    public void setGearRatio(double gearRatio){
        if (encoder != null)
            encoder.setGearRatio(gearRatio);
        else
            this.gearRatio = gearRatio;
            
    }

    public double getGearRatio(){
        if (encoder != null)
            return encoder.getGearRatio();
        else    
            return gearRatio;
    }

    public double getVoltage(){
        return motor.getMotorVoltage().getValueAsDouble();
    }
}
