package frc.robot.utils.motor;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;

import frc.robot.utils.encoder.Encoder;

public class ElevatorSimMotor implements Motor{
    private ElevatorSim motor; 
    private PIDController pid = new PIDController(0,0,0);
    private Encoder encoder;
    private double gearRatio;
    private double setPoint;
    private ElevatorFeedforward feedForward;

    private double velocityMPS = 0;
    private double positionMeters = 0;

    public ElevatorSimMotor(Encoder encoder, double gearRatio, PIDController pid, ElevatorFeedforward feedForward, ElevatorSim elevatorSim){
        this.encoder = encoder;
        this.gearRatio = gearRatio;
        this.pid = pid;
        this.feedForward = feedForward;
        this.motor = elevatorSim;        
    }

    public void setSpeed(double speed){
        motor.setInput(speed);
    }

    public double getSpeed(){
        return motor.getOutput(0);
    }

    public void setPosition(double position){
        positionMeters = position;
    }
    
    public double getPosition(){
        if(encoder == null){
            return motor.getPositionMeters();
        } else{
            return encoder.getPosition().getDegrees();
        }
    }        
    
    public void setSetpoint(double setPoint){
        double pos = pid.calculate(getPosition(), setPoint) + feedForward.calculate(motor.getVelocityMetersPerSecond()); //Needs velocity for feedforward
        positionMeters = setPoint;
        this.setPoint = setPoint;
    }
    
    public void periodic(){
        if (encoder != null){
            encoder.periodic();
            encoder.setPosition(Rotation2d.fromDegrees(positionMeters));
        }
        motor.update(0.020);
        motor.setState(positionMeters, velocityMPS);

        RoboRioSim.setVInVoltage(
            BatterySim.calculateDefaultBatteryLoadedVoltage(motor.getCurrentDrawAmps()));
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
        return motor.getCurrentDrawAmps();
    }
    public boolean isAtSetpoint(double deadzone){
        if (getPosition() >= setPoint - deadzone && getPosition() <= setPoint + deadzone)
            return true;
        else 
            return false;
    }
        
    public SimpleMotorFeedforward getFeedForward(){
        return new SimpleMotorFeedforward(feedForward.getKs(), feedForward.getKv(), feedForward.getKa());
    }

    public void setFeedFoward(double kS, double kV, double kA){
        feedForward = new ElevatorFeedforward(kS, feedForward.getKg(), kV, kA);
    }
}
