package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.motor.Motor;

public class Deepclimb extends SubsystemBase {
    
    public Motor climbMotor;

    private enum ClimbStates{
        Climb,
        Stow,
    }

    public Deepclimb(Motor climbMotor){
        this.climbMotor = climbMotor;
    }

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
