package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.motor.Motor;

public class Mechanisms extends SubsystemBase {

    private Motor elevator, elevator2;
    private Motor wrist;
    private double elevatorSetpoint = 0;
    private double wristSetpoint = 0;
    private boolean isManual = false;
    private double lowerLimit;
    private double upperLimit;
    private MechanismsState state = MechanismsState.Store;

    private enum MechanismsState{
        ReefL1,
        ReefL2,
        ReefL3,
        ReefL4,
        AlgaeL1,
        AlgaeL2,
        AlgaeProcessor,
        AlgaeBarge,
        Store,
        Intake,
        Climb,
    };

    public Mechanisms(Motor elevator, Motor elevator2, Motor wrist){
        this.elevator = elevator;
        this.elevator2 = elevator2;
        this.wrist = wrist;
    }
    
    public void setWristSetpoint(double setPoint) {
        wristSetpoint = setPoint;
        isManual = false;
    }

    public void setElevatorSetpoint(double setPoint) {
        elevatorSetpoint = setPoint;
        isManual = false;
    }

    public void setElevatorSpeed(double speed) {
        elevator.setSpeed(speed);
        elevator2.setSpeed(speed); 
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
                elevatorSetpoint = 0;
                wristSetpoint = 0;
                break;
                
            case ReefL2:
                elevatorSetpoint = 0;
                wristSetpoint = 0;
                break;

            case ReefL3:
                elevatorSetpoint = 0;
                wristSetpoint = 0;
                break;

            case ReefL4:
                elevatorSetpoint = 0;
                wristSetpoint = 0;
                break;

            case AlgaeL1:
                elevatorSetpoint = 0;
                wristSetpoint = 0;
                break;

            case AlgaeL2:
                elevatorSetpoint = 0;
                wristSetpoint = 0;
                break;

            case AlgaeProcessor:
                elevatorSetpoint = 0;
                wristSetpoint = 0;
                break;

            case AlgaeBarge:
                elevatorSetpoint = 0;
                wristSetpoint = 0;
                break;

            case Store:
                elevatorSetpoint = 0;
                wristSetpoint = 0;
                break;

            case Intake:
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
            elevator2.setSetpoint(elevatorSetpoint);
            wrist.setSetpoint(wristSetpoint);
        }

        elevator.periodic();
        elevator2.periodic();
        wrist.periodic();
        
        handleState();
    }
}
