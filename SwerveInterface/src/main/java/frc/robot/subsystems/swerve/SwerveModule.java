package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.swerve.modules.SwerveModuleType;

public class SwerveModule {
    public final int moduleNumber;
    public final SwerveModuleType module;
    private double timesRotated = 0;

    public SwerveModule(int moduleNumber, SwerveModuleType moduleType) {
        this.moduleNumber = moduleNumber;
        module = moduleType;

        module.configure();
        module.zeroWheels();
    }

    public void setState(SwerveModuleState state, boolean isOpenLoop) {
        // Prevents angle motor from turning further than it needs to. 
        // E.G. rotating from 10 to 270 degrees CW vs CCW.
        // System.out.println("Angle: " + state.angle.getRadians() + "Mod #: " + moduleNumber);

        state.optimize(getAngle());

        if (isOpenLoop) {
            module.setOpenDriveState(state);
        } else {
            module.setClosedDriveState(state);
        }
        module.setAngle(state.angle);
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(module.getDriveVelocity(), getAngle());
    }

    public Rotation2d getEncoder() {
        return module.getEncoder().getPosition();
    }

    public Rotation2d getAngle() {
        Rotation2d rot = module.getAngle();
        return rot;
    }

    public SwerveModulePosition getPosition() {
        SwerveModulePosition pos = new SwerveModulePosition(module.getDrivePosition(), getAngle());

        return pos;
    }
}
