package frc.robot.utils.motor;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.utils.encoder.Encoder;

public class ArmSimMotor implements Motor {

    private SingleJointedArmSim motor;
    private Encoder encoder;
    private PIDController pid;
    private ArmFeedforward feedforward;
    private double setpoint = 0;
    private double outputVoltage = 0;
    
    public ArmSimMotor(Encoder encoder, SingleJointedArmSim armSim, PIDController pid, ArmFeedforward feedforward){
        motor = armSim;
        this.encoder = encoder;
        this.pid = pid;
        this.feedforward = feedforward;
    }

    public void setSpeed(double speed){
        motor.setInput(speed);
    }

    public double getSpeed(){
        return motor.getOutput(0);
    }

    public void setPosition(double position){
        
    }
    
    public double getPosition(){
        if(encoder != null){
            return encoder.getPosition().getDegrees();
        }
        return Units.radiansToDegrees(motor.getAngleRads());
    }
    
    public void setSetpoint(double setPoint){
        outputVoltage = pid.calculate(getPosition(), setPoint) + feedforward.calculate(getPosition(), pid.getErrorDerivative());
        this.setpoint = setPoint;
        motor.setInputVoltage(outputVoltage);
    }

    /** Must be called by the subystems periodic method */
    public void periodic(){
        motor.update(0.02);
    }
    
    public void setPID(double p, double i, double d){
        pid = new PIDController(p, i, d);
    }
    
    public PIDController getPID(){
        return pid;
    }

    public Encoder getEncoder(){
        return encoder;
    }

    public double getVoltage(){
        return outputVoltage;
    }

    public boolean isAtSetpoint(double deadzone){
        return getPosition() <= setpoint + deadzone && getPosition() >= setpoint - deadzone;
    }

    public SimpleMotorFeedforward getFeedForward(){
        return new SimpleMotorFeedforward(feedforward.getKs(), feedforward.getKv(), feedforward.getKa());
    }

    public void setFeedFoward(double kS, double kV, double kA){
        feedforward = new ArmFeedforward(kS, feedforward.getKg(), kV, kA);
    }
}
