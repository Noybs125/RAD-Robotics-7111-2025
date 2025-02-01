package frc.robot.utils.motor;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.utils.encoder.Encoder;


public class REVMotor implements Motor {
    private SparkMax motor;
    private PIDController pid;
    private Encoder encoder = null;
    private double gearRatio = 1;
    private SimpleMotorFeedforward feedForward;
    private double setPoint;
    private Motor simType; 

    public REVMotor (int id) {
        motor = new SparkMax(id,MotorType.kBrushless);

    }
    
    public REVMotor(int id, Encoder encoder, double gearRatio, PIDController pid, SimpleMotorFeedforward feedforward, Motor simType){
        this.encoder = encoder;
        this.gearRatio = gearRatio;
        this.pid = pid;
        this.feedForward = feedforward;

        motor = new SparkMax(id, MotorType.kBrushless);
        this.simType = simType;
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
            motor.getEncoder().setPosition(position / gearRatio);
        }
    }
    
    public double getPosition(){
        if(encoder == null){
            return motor.getEncoder().getPosition() * gearRatio;
        } else{
            return encoder.getPosition().getDegrees();
        }
    }  
    
    public void setSetpoint(double setPoint){
        double pidOutput = pid.calculate(getPosition(), setPoint);
        double feedforwardOutput = feedForward.calculate(pid.getErrorDerivative());
        motor.setVoltage(pidOutput + feedforwardOutput); //Needs velocity for feedforward
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

    public Encoder getEncoder(){
        return encoder;
    }

    public double getVoltage(){
        return motor.getBusVoltage();
    }

    public boolean isAtSetpoint(double deadzone){
        if (getPosition() >= setPoint - deadzone && getPosition() <= setPoint + deadzone)
            return  true;
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
