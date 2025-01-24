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
        
    }
}
