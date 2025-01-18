package frc.robot.utils.motor;

import com.revrobotics.spark.SparkMax;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.utils.encoder.Encoder;


public class REVMotor implements Motor {
    private SparkMax motor;
    private PIDController pid;
    private Encoder encoder = null;
    private double gearRatio = 1;

    public REVMotor (int id) {
        motor = new SparkMax(id,MotorType.kBrushless);

    }
    
    public REVMotor(int id, Encoder encoder){
        this.encoder = encoder;

        motor = new SparkMax(id, MotorType.kBrushless);
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
            motor.getEncoder().setPosition(position * gearRatio);
        }
    }
    
    public double getPosition(){
        if(encoder == null){
            return motor.getEncoder().getPosition() / gearRatio;
        } else{
            return encoder.getPosition().getDegrees();
        }
    }  
    
    public void setSetpoint(double setPoint){
        motor.set(pid.calculate(getPosition(), setPoint));
    }

    public void periodic(){}

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
        if(encoder != null) {
            encoder.setGearRatio(gearRatio);
        } else {
            this.gearRatio = gearRatio;
        }
    };

    public double getGearRatio(){
        if(encoder != null){
            return encoder.getGearRatio();
        }else{
            return gearRatio;
        }
    };

    public double getVoltage(){
        return motor.getBusVoltage();
    };
    
}
