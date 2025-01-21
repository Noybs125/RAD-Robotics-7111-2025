package frc.robot.utils.motor;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.utils.encoder.Encoder;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

public class ElevatorSimMotor implements Motor{
    private ElevatorSim motor; 
    private PIDController pid = new PIDController(0,0,0);
    private Encoder encoder = null;
    private double gearRatio;
    private double setPoint;
    private SimpleMotorFeedforward feedForward;
    private double kV;
    private double kA;

    public ElevatorSimMotor(Encoder encoder, double gearRatio, PIDController pid, SimpleMotorFeedforward feedForward, double kV, double kA, DCMotor gearbox, double minheight, double maxheight, double startheight, double[] measureStdDevs){
        this.encoder = encoder;
        this.gearRatio = gearRatio;
        this.pid = pid;
        this.feedForward = feedForward;
        this.kV = kV;
        this.kA = kA;
        motor = new ElevatorSim(kV, kA, gearbox, minheight, maxheight, true, startheight, measureStdDevs);

    }

    

    public void setSpeed(double speed){
        motor.setInput(speed);
    }

    public double getSpeed(){
        return motor.getOutput(0);
    }

    public void setPosition(double position){
        if(encoder != null){
            encoder.setPosition(Rotation2d.fromDegrees(position));
        }
        else {
            motor.setState(position,0);
        }
    }
    
    public double getPosition(){
        if(encoder == null){
            return motor.getPositionMeters();
        } else{
            return encoder.getPosition().getDegrees();
        }
    }        
    
    public void setSetpoint(double setPoint){
        motor.setInput(pid.calculate(getPosition(), setPoint));
        this.setPoint = setPoint;
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
        return motor.getCurrentDrawAmps();
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
