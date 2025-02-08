package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Mechanisms.MechanismsState;
import com.ctre.phoenix6.mechanisms.MechanismState;

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
        defaultState,
    }

    public SuperStructure(Swerve swerve, Vision vision, Sensors sensors, Mechanisms mechanisms, Flywheels flywheels){
        this.swerve = swerve;
        this.vision = vision;
        this.mechanisms = mechanisms;
        this.flywheels = flywheels;

        controlRobotState = ControlState.Default;
        actualRobotState = ActualState.defaultState;
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
                coralL1();
                break;
            case coralL2:
                coralL2();
                break;
            case coralL3:
                coralL3();
                break;
            case coralL4:
                coralL4();
                break;
            case coralFeeder:
                coralFeeder();
                break;
            case algaeProcessor:
                algaeProcessor();
                break;
            case algaeNet:
                algaeNet();
                break;
            case algaeL2:
                algaeL2();
                break;
            case algaeL3:
                algaeL3();
                break;
            case deepClimb:
                deepClimb();
                break;
            case defaultState:
                defaultState();
                break;
        }
    }

    private void setRobotState(ControlState state){
        controlRobotState = state;
    }

    public Command setRobotStateCommand(ControlState state){
        return new InstantCommand(() -> setRobotState(state));
    }

    private void coralL1(){
        mechanisms.setState(Mechanisms.MechanismsState.ReefL1);
    }
    private void coralL2(){
        mechanisms.setState(Mechanisms.MechanismsState.ReefL2);
    }
    private void coralL3(){
        mechanisms.setState(Mechanisms.MechanismsState.ReefL3);
    }
    private void coralL4(){
        mechanisms.setState(Mechanisms.MechanismsState.ReefL4);
    }
    private void coralFeeder(){
        mechanisms.setState(Mechanisms.MechanismsState.CoralFeeder);
    }
    private void algaeProcessor(){
        mechanisms.setState(Mechanisms.MechanismsState.AlgaeProcessor);
    }
    private void algaeNet(){
        mechanisms.setState(Mechanisms.MechanismsState.AlgaeNet);
    }
    private void algaeL2(){
        mechanisms.setState(Mechanisms.MechanismsState.AlgaeL2);
    }
    private void algaeL3(){
        mechanisms.setState(Mechanisms.MechanismsState.AlgaeL3);
    }
    private void deepClimb(){
        mechanisms.setState(Mechanisms.MechanismsState.Climb);
    }
    private void defaultState(){
        mechanisms.setState(Mechanisms.MechanismsState.Store);
    }
}
