package frc.robot.utils.motor;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;


public class REVMotor implements Motor {
    private SparkMax motor;
    private PIDController pid;

    public REVMotor (int id) {
        motor = new SparkMax(id,MotorType.kBrushless);

    }

    public void setSpeed(double speed){
        motor.set(speed);
    }


    public double getSpeed(){
        return motor.get();
    }
    

    public void setPosition(double position){
        motor.getEncoder().setPosition(position);
    }

    
    public double getPosition(){
       return motor.getEncoder().getPosition();
    }
        
    
    public void setSetpoint(double setPoint){
        motor.set(pid.calculate(getPosition(), setPoint));
    }

    
    public void periodic(){}
    
}
