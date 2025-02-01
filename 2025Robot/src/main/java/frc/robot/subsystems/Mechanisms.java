package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.motor.ElevatorSimMotor;
import frc.robot.utils.motor.Motor;

public class Mechanisms extends SubsystemBase {

    private Motor elevator;
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

    public Mechanisms(Motor elevator, Motor wrist){
        this.elevator = elevator;

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
            wrist.setSetpoint(wristSetpoint);
        }

        elevator.periodic();
        wrist.periodic();
        
        handleState();
    }

    public void simulationPeriodic(){
        double elevatorMinimumLength = 0;
        Mechanism2d mech2d = new Mechanism2d(20, 50);
        MechanismRoot2d mech2dRoot = mech2d.getRoot("Elevator Root", 10, 0);
        MechanismRoot2d root = mech2d.getRoot("climber", 2, 0);
        MechanismLigament2d elevatorMech2d = root.append(new MechanismLigament2d("elevator", elevatorMinimumLength, 90));

        elevatorMech2d = mech2dRoot.append(new MechanismLigament2d("Elevator", elevator.getPosition(), 90));
        elevatorMech2d.setLength(elevator.getPosition());

        SmartDashboard.putData("Mech2d", mech2d);
        SmartDashboard.putData("Elevator Sim", mech2d);
    }
}
