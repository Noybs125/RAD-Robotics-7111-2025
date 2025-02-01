package frc.robot.utils.motor;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.utils.encoder.Encoder;

public class BagMotor implements Motor{
    private VictorSPX motor; 
    private PIDController pid = new PIDController(0,0,0);
    private Encoder encoder = null;
    private double gearRatio;
    private double setPoint;
    private SimpleMotorFeedforward feedForward;

    public BagMotor(int id){
        motor = new VictorSPX(id);
    }
    
    public BagMotor(int id, Encoder encoder, double gearRatio, PIDController pid, SimpleMotorFeedforward feedForward){
        this.encoder = encoder;
        this.gearRatio = gearRatio;
        this.pid = pid;
        this.feedForward = feedForward;

        motor = new VictorSPX(id);
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
        this.setPoint = setPoint;
    }
    
    public void periodic(){
        if (encoder != null){
            encoder.periodic();
        }
    }
    
    public void setPID(double p, double i, double d){
        pid.setPID(p, i, d);
    }

    public PIDController getPID(){
        return pid;
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
        if (getPosition() >= setPoint + deadzone && getPosition() <= setPoint + deadzone)
            return true;
        else 
            return false;
    }
        
    public SimpleMotorFeedforward getFeedForward(){
        return feedForward;
    }

    public void setFeedFoward(double kS, double kV, double kA){
        feedForward = new SimpleMotorFeedforward(kS, kV, kA);
    }
}
