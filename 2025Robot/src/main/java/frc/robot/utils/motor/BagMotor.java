package frc.robot.utils.motor;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.utils.encoder.Encoder;

public class BagMotor implements Motor{
    private VictorSPX motor; 
    private PIDController pid = new PIDController(0,0,0);
    private Encoder encoder = null;
    private double gearRatio;

    public BagMotor(int id){
        motor = new VictorSPX(id);
    }
    
    public BagMotor(int id, Encoder encoder){
        this.encoder = encoder;

        motor = new VictorSPX(id);
    }

    public BagMotor(int id, double gearRatio){
        motor = new VictorSPX(id);
        this.gearRatio = gearRatio;
    }

    public BagMotor(int id, Encoder encoder, double gearRatio){
        motor = new VictorSPX(id);
        this.encoder = encoder;
        this.gearRatio = gearRatio;
    }

    public void setSpeed(double speed){
        motor.set(VictorSPXControlMode.PercentOutput, speed);
    }

    public double getSpeed(){
        return motor.getMotorOutputPercent();
    }

    public void setPosition(double position){
        if(encoder != null){
            encoder.setPosition(Rotation2d.fromDegrees(position));
        }
    }
    
    public double getPosition(){
        if(encoder == null){
            return 0;
        } else{
            return encoder.getPosition().getDegrees();
        }
    }        
    
    public void setSetpoint(double setPoint){
        motor.set(VictorSPXControlMode.PercentOutput, pid.calculate(getPosition(), setPoint));
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
        return motor.getBusVoltage();
    }
    public boolean isAtSetpoint(double deadzone){
    }
        
    public double getFeedFoward(){

    }

    public void setFeedFoward(double kS, double kV, double kA){

    }
}
