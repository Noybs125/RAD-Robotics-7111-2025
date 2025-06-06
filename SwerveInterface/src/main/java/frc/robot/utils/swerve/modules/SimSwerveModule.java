package frc.robot.utils.swerve.modules;


import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.utils.encoder.Encoder;
import frc.robot.utils.swerve.DrivebaseConfig;
import frc.robot.utils.swerve.SwerveModuleConstants;

public class SimSwerveModule implements SwerveModuleType{

    private DCMotorSim driveMotorSim;
    private DCMotorSim angleMotorSim;

    public SimSwerveModule(SwerveModuleConstants constants){
        driveMotorSim = new DCMotorSim(LinearSystemId.createDCMotorSystem(constants.driveMotor, getDriveVelocity(), getDrivePosition()));
        angleMotorSim = new DCMotorSim(null, angleMotor, null);
    }

    @Override
    public void setOpenDriveState(SwerveModuleState state) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setOpenDriveState'");
    }

    @Override
    public void setClosedDriveState(SwerveModuleState state) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setClosedDriveState'");
    }

    @Override
    public double getDriveVelocity() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getDriveVelocity'");
    }

    @Override
    public double getDrivePosition() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getDrivePosition'");
    }

    @Override
    public Rotation2d getAngle() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getAngle'");
    }

    @Override
    public void setAngle(Rotation2d rotation) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setAngle'");
    }

    @Override
    public Encoder getEncoder() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getEncoder'");
    }

    @Override
    public void zeroWheels() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'zeroWheels'");
    }

    @Override
    public void configure() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'configure'");
    }
    
}
