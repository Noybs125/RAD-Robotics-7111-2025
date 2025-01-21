package frc.robot.utils.motor;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix6.controls.PositionVoltage;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.utils.encoder.Encoder;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;

public class CTREMotor implements Motor {
    private TalonFX motor;
    PIDController pid = new PIDController(0, 0, 0);
    private TalonFXConfiguration config;
    private Encoder encoder = null;
    private double gearRatio;
    private double currentSetpoint;
    private SimpleMotorFeedforward feedforward;

    
    
    public CTREMotor(int id, Encoder encoder, double gearRatio, PIDController pid, double setPoint, SimpleMotorFeedforward feedForward){
        this.encoder = encoder;
        this.gearRatio = gearRatio;
        this.pid = pid;
        this.currentSetpoint = setPoint;
        this.feedforward = feedForward;
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
        currentSetpoint = setPoint;
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
    public boolean isAtSetpoint(double deadzone){
        if(getPosition() >= currentSetpoint - deadzone && getPosition() <= currentSetpoint + deadzone){
            return true;
        }
        return false;
    }
        
    public SimpleMotorFeedforward getFeedForward(){
        return feedforward;
    }

    public void setFeedFoward(double kS, double kV, double kA){
        config.Slot0.kS = kS;
        config.Slot0.kV = kV;
        config.Slot0.kA = kA;
        motor.getConfigurator().apply(config);
        feedforward = new SimpleMotorFeedforward(kS, kV, kA);
    }
}
