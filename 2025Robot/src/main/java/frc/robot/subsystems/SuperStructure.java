package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class SuperStructure extends SubsystemBase {
    private Swerve swerve;
    private Vision vision;
    private Mechanisms mechanisms;
    private Flywheels flywheels; 
    private RobotState currentRobotState;
    private RobotState robotState;

    public enum RobotState {
        ReafL1Processor,
        ReafL2,
        ReafL3,
        ReafL4Net,
        ReafFeeder,
        DeepClimb,
        Default,
    }

    public SuperStructure(Swerve swerve, Vision vision, Sensors sensors, Mechanisms mechanisms, Flywheels flywheels){
        this.swerve = swerve;
        this.vision = vision;
        this.mechanisms = mechanisms;
        this.flywheels = flywheels;
    }


    public void periodic() {
        switch (robotState) {   
            case ReafL1Processor:
                //checks for whether it should align for reaf or processor
                break;
            case ReafL2:
                //checks for whether it should score coral or intake algae
                break;
            case ReafL3:
                //checks for whether it should score coral or intake algae
                break;
            case ReafL4Net:
                //checks for whether it should score coral on L4 or score algae in net
                break;
            case ReafFeeder:
                //
                break;
            case DeepClimb:
                //
                break;
            case Default:
            default:
                //will contain the default state
                break;
        }
    }

    private void setRobotState(RobotState state){
        robotState = state;
    }

    public Command setRobotStateCommand(RobotState state){
        return new InstantCommand(() -> setRobotState(state));
    }

    private void coralL1(){}
    private void coralL2(){}
    private void coralL3(){}
    private void coralL4(){}
    private void coralFeeder(){}
    private void algaeProcessor(){}
    private void algaeNet(){}
    private void algaeL2(){}
    private void algaeL3(){}
    private void deepClimb(){}
    private void defaultState(){}
}
