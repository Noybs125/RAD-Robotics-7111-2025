package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.motor.CTREMotor;
import frc.robot.utils.motor.Motor;

public class Deepclimb extends SubsystemBase {
    
    public CTREMotor climbMotor;
    public TalonFXConfiguration climbMotorConfigs;

    private ClimbStates climbstate;
    /**
     * State of robot climb winch, detailing either in the operating state "Climb", and the stowed state "Stow".
     * States include: "Climb" and "Stow".
     */
    public enum ClimbStates{
        Retracted,
        Extended,
    }

    /**
     * Initilizes the end effector motor object and constructor to class "Deepclimb".
     * @param climbMotor -Motor that operates winch for the Deepclimb mechanism.
     */
    public Deepclimb(){
        this.climbMotorConfigs = new TalonFXConfiguration();
        climbMotorConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        this.climbMotor = new CTREMotor(5, null, 64, Constants.kMechanisms.deepClimbPID, Constants.kMechanisms.deepClimbFF, null, climbMotor, climbMotorConfigs);
    }

    /**
     * Sets state of Deepclimb, either being stowed or climbing.
     * @param state -Type "ClimbStates", either raises or loweres winch for climb.
     */
    public void setState(ClimbStates state){
        switch (state) {
            case Retracted:
                climbMotor.setSetpoint(0, false);
                climbstate = state;
                break;
            
            case Extended:
                climbMotor.setSetpoint(0, false);
                climbstate = state;
                break;
            default:
                break;
        }
    }
    public void setSpeed(double speed) {
        climbMotor.setSpeed(speed);
    }

    public void periodic()
    {
        SmartDashboard.putNumber("DeepClimbPose", climbMotor.getPosition());
    }
}
