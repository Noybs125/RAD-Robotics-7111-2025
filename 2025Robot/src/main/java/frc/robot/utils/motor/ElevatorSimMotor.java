package frc.robot.utils.motor;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.Constants;
import frc.robot.utils.encoder.Encoder;
import frc.robot.utils.encoder.WpiEncoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.PWMSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;

public class ElevatorSimMotor implements Motor{
    private ElevatorSim motor; 
    private PIDController pid = new PIDController(0,0,0);
    private Encoder encoder;
    private double gearRatio;
    private double setPoint;
    private SimpleMotorFeedforward feedForward;
    final Mechanism2d mech2d = new Mechanism2d(20, 50);
    final MechanismRoot2d mech2dRoot = mech2d.getRoot("Elevator Root", 10, 0);
    final MechanismLigament2d elevatorMech2d;

    public ElevatorSimMotor(Encoder encoder, double gearRatio, PIDController pid, SimpleMotorFeedforward feedForward, ElevatorSim elevatorSim){
        this.encoder = encoder;
        this.gearRatio = gearRatio;
        this.pid = pid;
        this.feedForward = feedForward;
        //The line below may not be needed
        //encoder = new WpiEncoder(Constants.kSimulation.kEncoderAChannel, Constants.kSimulation.kEncoderBChannel);
        this.motor = elevatorSim;
        elevatorMech2d = mech2dRoot.append(
            new MechanismLigament2d("Elevator", motor.getPositionMeters(), 90));

        SmartDashboard.putData("Elevator Sim", mech2d);
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
        var pos = pid.calculate(getPosition(), setPoint);
        motor.setInput(pos);
        this.setPoint = setPoint;
    }
    
    public void periodic(){
        if (encoder != null){
            encoder.periodic();
        }
        motor.update(0.020);
        motor.setState(motor.getPositionMeters(), motor.getVelocityMetersPerSecond());

        RoboRioSim.setVInVoltage(
            BatterySim.calculateDefaultBatteryLoadedVoltage(motor.getCurrentDrawAmps()));

        elevatorMech2d.setLength(encoder.getPosition().getRotations());
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
