package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.motor.Motor;

public class Deepclimb extends SubsystemBase {
    
    public Motor climbMotor;

    /**
     * State of robot climb winch, detailing either in the operating state "Climb", and the stowed state "Stow".
     * States include: "Climb" and "Stow".
     */
    private enum ClimbStates{
        Climb,
        Stow,
    }

    /**
     * Initilizes the end effector motor object and constructor to class "Deepclimb".
     * @param climbMotor -Motor that operates winch for the Deepclimb mechanism.
     */
    public Deepclimb(Motor climbMotor){
        this.climbMotor = climbMotor;
    }

    /**
     * Sets state of Deepclimb, either being stowed or climbing.
     * @param state -Type "ClimbStates", either raises or loweres winch for climb.
     */
    public void setState(ClimbStates state){
        switch (state) {
            case Climb:
                climbMotor.setSpeed(.5);
                break;
            
            case Stow:
                climbMotor.setSpeed(-.5);
                break;
            default:
                break;
        }
    }
}
