package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class SuperStructure extends SubsystemBase {
    private Swerve swerve;
    private Vision vision;
    private Mechanisms mechanisms;

    public SuperStructure(Swerve swerve, Vision vision, Mechanisms mechanisms){
        this.swerve = swerve;
        this.vision = vision;
        this.mechanisms = mechanisms;
    }

    public enum RobotState {
        CoralL1,
        CoralL2,
        CoralL3,
        CoralL4,
        CoralFeeder
    }

    //private RobotState previousRobotState;
    private RobotState currentRobotState;
    private RobotState robotState;


    public void periodic() {
        //previousRobotState = currentRobotState;
        switch (robotState) {   
            case CoralL1:
                break;
            case CoralL2:
                break;
            case CoralL3:
                break;
            case CoralL4:
                break;
            case CoralFeeder:
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
    private void deepclimb(){}
}
