package frc.robot.subsystems;

import com.fasterxml.jackson.annotation.JsonTypeInfo.Id;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.encoder.Encoder;
import frc.robot.utils.encoder.RevEncoder;
import frc.robot.utils.encoder.WpiEncoder;
import frc.robot.utils.motor.ArmSimMotor;
import frc.robot.utils.motor.CTREMotor;
import frc.robot.utils.motor.ElevatorSimMotor;
import frc.robot.utils.motor.Motor;

public class Mechanisms extends SubsystemBase {

    private Motor elevator;
    private Motor wrist;
    private Encoder testEncoder;
    private double elevatorSetpoint = 0;
    private double wristSetpoint = 0;
    private boolean isManual = false;
    private double lowerLimit;
    private double upperLimit;
    private MechanismsState state = MechanismsState.Store;

    public enum MechanismsState{
        ReefL1,
        ReefL2,
        ReefL3,
        ReefL4,
        AlgaeL2,
        AlgaeL3,
        AlgaeProcessor,
        AlgaeNet,
        Store,
        CoralFeeder,
        Climb,
    };

    private double elevatorMinimumLength = 0.14925;
    private double wristLength = 1;
    private Mechanism2d mech2d = new Mechanism2d(5, 5);
    private MechanismRoot2d elevatorMech2dRoot = mech2d.getRoot("Elevator Root", 1, 1);
    private MechanismRoot2d wristMech2dRoot = mech2d.getRoot("Wrist Root", 2, 1);
    private MechanismLigament2d elevatorMech2d = elevatorMech2dRoot.append(new MechanismLigament2d("elevator", elevatorMinimumLength, 90));
    private MechanismLigament2d wristMech2d = wristMech2dRoot.append(new MechanismLigament2d("wrist", wristLength, 0));

    public Mechanisms(){
        testEncoder = new WpiEncoder(0, 1);
        elevator = new ElevatorSimMotor(
            null, Constants.kSimulation.elevatorSimGearRatio, Constants.kSimulation.elevatorPid,
            Constants.kSimulation.elevatorFF, Constants.kSimulation.elevatorSimConstants
            );
        wrist =  new ArmSimMotor(null, Constants.kSimulation.armSim, Constants.kSimulation.wristPid,  null);//Constants.kSimulation.wristFF));
    }
    
    public void setWristSetpoint(double setPoint) {
        wristSetpoint = setPoint;
        isManual = false;
    }

    public void setElevatorSetpoint(double setPoint) {
        elevatorSetpoint = setPoint;
        isManual = false;
    }
    public double getElevatorHeight(){
        double elevatorHeight = (Math.PI * 2 * elevator.getPosition() / 360.0) * 12;
        return elevatorHeight;
    }

    public void setElevatorSpeed(double speed) {
        elevator.setSpeed(speed);
        isManual = true;
    }

    public void setWristSpeed(double speed) {
        wrist.setSpeed(speed);
        isManual = true;
    }

    public void setAllMechanismsSetpoint(double wristSetpoint, double elevatorSetpoint) {
        this.elevatorSetpoint = elevatorSetpoint;
        this.wristSetpoint = wristSetpoint;
        isManual = false;
    }

    public void setState(MechanismsState state){
        this.state = state;
    }

    public void moveElevThenArm(double elevatorSetpoint, double wristSetpoint, double deadzone){
        this.elevatorSetpoint = elevatorSetpoint;
        if(elevator.isAtSetpoint(deadzone)){
            this.wristSetpoint = wristSetpoint;
        }
    }

    public void moveArmThenElev(double elevatorSetpoint, double wristSetpoint, double deadzone){
        this.wristSetpoint = wristSetpoint;
        if(wrist.isAtSetpoint(deadzone)){
            this.elevatorSetpoint = elevatorSetpoint;
        }
    } 

    private void handleState() {
        switch (state) {
            case ReefL1:
                elevatorSetpoint = 0.16;
                wristSetpoint = 55;
                break;
                
            case ReefL2:
                elevatorSetpoint = 0.9;
                wristSetpoint = 215;
                break;

            case ReefL3:
                elevatorSetpoint = 1.6;
                wristSetpoint = 110;
                break;

            case ReefL4:
                elevatorSetpoint = 0;
                wristSetpoint = 0;
                break;

            case AlgaeL2:
                elevatorSetpoint = 0;
                wristSetpoint = 0;
                break;

            case AlgaeL3:
                elevatorSetpoint = 0;
                wristSetpoint = 0;
                break;

            case AlgaeProcessor:
                elevatorSetpoint = 0;
                wristSetpoint = 0;
                break;

            case AlgaeNet:
                elevatorSetpoint = 0;
                wristSetpoint = 0;
                break;

            case Store:
                elevatorSetpoint = 0;
                wristSetpoint = 0;
                break;

            case CoralFeeder:
                elevatorSetpoint = 0;
                wristSetpoint = 0;
                break;

            case Climb:
                elevatorSetpoint = 0;
                wristSetpoint = 0;
                break;
        
            default:
                elevatorSetpoint = 0;
                wristSetpoint = 0;
                break;
        }
    }
    public void periodic() {


        if (isManual) {
            
        }
        else {
            elevator.setSetpoint(elevatorSetpoint);
            wrist.setSetpoint(wristSetpoint);
        }

        elevator.periodic();
        wrist.periodic();
        
        handleState();
    }

    public void simulationPeriodic(){
        elevatorMech2d.setLength(elevator.getPosition());
        wristMech2d.setAngle(wrist.getPosition());

        SmartDashboard.putData("Mech2d", mech2d);
        SmartDashboard.putNumber("Elevator Pos", elevator.getPosition());
        SmartDashboard.putNumber("Wrist", wrist.getPosition());
    }
}
