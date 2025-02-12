package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.motor.Motor;
import frc.robot.utils.motor.REVMotor;

public class Flywheels extends SubsystemBase {

    public Motor effectorWheels;
    
    private enum WheelsStates{
        CoralIntake,
        AlgaeIntake,
        CoralOutake,
        AlgaeOutake
    };

    public Flywheels(){
         effectorWheels = new REVMotor(0);
    }

    public void setSpeed(double speed){
        effectorWheels.setSpeed(speed);
    }

    public double getSpeed(){
        return effectorWheels.getSpeed();
    }
    
    public void setSpeed(WheelsStates speed){
        switch (speed) {
            case CoralIntake:
                effectorWheels.setSpeed(.5);
                break;
        
            case AlgaeIntake:
                effectorWheels.setSpeed(.5);
                break;

            case CoralOutake:
                effectorWheels.setSpeed(-.5);
                break;
            
            case AlgaeOutake:
                effectorWheels.setSpeed(-.5);
                break;

            default:
                break;
        }
    }
}
