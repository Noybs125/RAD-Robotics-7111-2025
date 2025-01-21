package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SuperStructure extends SubsystemBase {
    private Swerve swerve;
    private Vision vision;

    public SuperStructure(Swerve swerve, Vision vision){
        this.swerve = swerve;
        this.vision = vision;
    }

    public enum RobotState {
        CoralL1,
        CoralL2,
        CoralL3,
        CoralL4,
        CoralFeeder
    }
    
}
