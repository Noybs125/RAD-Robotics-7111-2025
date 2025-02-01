package frc.robot.utils.motor;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.utils.encoder.Encoder;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

public class CTREMotor implements Motor {
    private TalonFX motor;
    PIDController pid = new PIDController(0, 0, 0);
    private TalonFXConfiguration config;
    private Encoder encoder = null;
    private double gearRatio;
    private double currentSetpoint;
    private SimpleMotorFeedforward feedforward;
    
    public CTREMotor(int id, Encoder encoder, double gearRatio, PIDController pid, SimpleMotorFeedforward feedForward){
        this.encoder = encoder;
        this.gearRatio = gearRatio;
        this.pid = pid;
        this.feedforward = feedForward;
        motor = new TalonFX(id);
    }

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
        double pidOutput = pid.calculate(encoder.getPositionAsDouble());
        double feedforwardOutput = feedforward.calculate(pid.getErrorDerivative());
        motor.setVoltage(pidOutput + feedforwardOutput); //Needs velocity for feedforward
        currentSetpoint = setPoint;
    }

    public void periodic(){
        if (encoder != null){
            encoder.periodic();
        }
    }

    public void setPID(double p, double i, double d){
        pid.setPID(p, i, d);
    }

    public PIDController getPID() {
        return pid;
    }

    public Encoder getEncoder(){
        return encoder;
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
