package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class SuperStructure extends SubsystemBase {
    private Swerve swerve;
    private Vision vision;
    private Mechanisms mechanisms;
    private Flywheels flywheels; 
    private ActualState actualRobotState;
    private ControlState controlRobotState;

    public enum ControlState {
        ReafL1Processor,
        ReafL2,
        ReafL3,
        ReafL4Net,
        ReafFeeder,
        DeepClimb,
        Default,
    }

    public enum ActualState {
        coralL1,
        coralL2,
        coralL3,
        coralL4,
        coralFeeder,
        algaeProcessor,
        algaeNet,
        algaeL2,
        algaeL3,
        deepClimb,
        DefaultState,
    }

    public SuperStructure(Swerve swerve, Vision vision, Sensors sensors, Mechanisms mechanisms, Flywheels flywheels){
        this.swerve = swerve;
        this.vision = vision;
        this.mechanisms = mechanisms;
        this.flywheels = flywheels;

        controlRobotState = ControlState.Default;
        actualRobotState = ActualState.DefaultState;
    }


    public void periodic() {
        switch (controlRobotState) {   
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

    private void actualState()
    {
        switch(actualRobotState)
        {
            case coralL1:
                break;
            case coralL2:
                break;
            case coralL3:
                break;
            case coralL4:
                break;
            case coralFeeder:
                break;
            case algaeProcessor:
                break;
            case algaeNet:
                break;
            case algaeL2:
                break;
            case algaeL3:
                break;
            case deepClimb:
                break;
            case DefaultState:
                break;
        }
    }

    private void setRobotState(ControlState state){
        controlRobotState = state;
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
