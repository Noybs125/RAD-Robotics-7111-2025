package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Mechanisms.MechanismsState;
import frc.robot.subsystems.Swerve.SwerveState;

import com.ctre.phoenix6.mechanisms.MechanismState;

import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class SuperStructure extends SubsystemBase {
    private Swerve swerve;
    private Vision vision;
    private Field field;
    private Sensors sensors;
    private Mechanisms mechanisms;
    private Flywheels flywheels; 
    private ActualState actualRobotState;
    private ControlState controlRobotState;

    public enum ControlState {
        ReefL1Processor,
        ReefL2,
        ReefL3,
        ReefL4Net,
        ReefFeeder,
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

    public SuperStructure(Swerve swerve, Vision vision, Field field, Sensors sensors, Mechanisms mechanisms, Flywheels flywheels){
        this.swerve = swerve;
        this.vision = vision;
        this.field = field;
        this.sensors = sensors;
        this.mechanisms = mechanisms;
        this.flywheels = flywheels;

        controlRobotState = ControlState.Default;
        actualRobotState = ActualState.defaultState;
    }


    public void periodic() {
        manageControlState();
        manageActualState();


    }

    private void manageControlState(){
        switch (controlRobotState) {   
            case ReefL1Processor:
                //checks for whether it should align for reef or processor
                actualRobotState = ActualState.coralL1;
                break;
            case ReefL2:
                //checks for whether it should score coral or intake algae
                actualRobotState = ActualState.coralL2;
                break;
            case ReefL3:
                //checks for whether it should score coral or intake algae
                actualRobotState = ActualState.coralL3;
                break;
            case ReefL4Net:
                //checks for whether it should score coral on L4 or score algae in net
                boolean isNet = false; //true if we have algae
                actualRobotState = isNet
                    ? ActualState.algaeNet
                    : ActualState.coralL4;
                break;
            case ReefFeeder:
                //
                actualRobotState = ActualState.coralFeeder;
                break;
            case DeepClimb:
                //
                actualRobotState = ActualState.deepClimb;
                break;
            case Default:
            default:
                actualRobotState = ActualState.defaultState;
                break;
        }
    }

    private void manageActualState()
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
        return runOnce(() -> setRobotState(state));
    }

    private void coralL1(){
        //mechanisms.setState(Mechanisms.MechanismsState.ReefL1);
        swerve.setState(SwerveState.Vision);
    }
    private void coralL2(){
        mechanisms.setState(Mechanisms.MechanismsState.ReefL2);
        if(vision.isAtTarget(0, null, null, 0));
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
        //mechanisms.setState(Mechanisms.MechanismsState.Store);
        swerve.setState(SwerveState.DefaultState);
    }
}
