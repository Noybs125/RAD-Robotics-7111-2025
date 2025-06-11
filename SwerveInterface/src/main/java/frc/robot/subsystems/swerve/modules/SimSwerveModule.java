package frc.robot.subsystems.swerve.modules;


import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.swerve.DrivebaseConfig;
import frc.robot.subsystems.swerve.SwerveModuleConstants;
import frc.robot.utils.encoder.Encoder;

public class SimSwerveModule implements SwerveModuleType{
    private DCMotor driveMotorOutput;
    private DCMotor angleMotorOutput;
    private DCMotorSim driveMotorSim;
    private DCMotorSim angleMotorSim;

    private Encoder encoder;

    private double driveMotorAmps;
    private double angleMotorAmps;
    private PIDController drivePID;
    private PIDController anglePID;

    public SimSwerveModule(SwerveModuleConstants constants){
        driveMotorOutput = constants.driveMotor.dcMotor;
        angleMotorOutput = constants.angleMotor.dcMotor;

        driveMotorSim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(driveMotorOutput, 0.0257, 1/constants.driveMotor.gearRatio), 
            driveMotorOutput);
        angleMotorSim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(angleMotorOutput, 0.002, 1/constants.angleMotor.gearRatio), 
            angleMotorOutput);

        encoder = constants.encoder;

        driveMotorAmps = constants.driveMotor.currentLimit; //TODO is there some formula/estimation?
        angleMotorAmps = constants.angleMotor.currentLimit;
        anglePID = constants.angleMotor.pid;
        drivePID = constants.angleMotor.pid;
    }

    @Override
    public void setOpenDriveState(SwerveModuleState state) {
        
    }

    @Override
    public void setClosedDriveState(SwerveModuleState state) {
        double torque = driveMotorOutput.getTorque(driveMotorAmps);
        double speedRadPerSec = Units.rotationsToRadians(state.speedMetersPerSecond / SwerveConstants.wheelDiameter);
        driveMotorSim.setInputVoltage(drivePID.calculate(getDriveVelocity(), speedRadPerSec));
    }

    @Override
    public double getDriveVelocity() {
        //System.out.println("getting drive velocity");
        return (driveMotorSim.getAngularVelocityRadPerSec() * SwerveConstants.wheelDiameter);
    }

    @Override
    public double getDrivePosition() {

        return driveMotorSim.getAngularPositionRotations();
    }

    @Override
    public Rotation2d getAngle() {
        System.out.println(angleMotorSim.getAngularPositionRotations());
        return Rotation2d.fromRotations(angleMotorSim.getAngularPositionRotations());
    }

    @Override
    public void setAngle(Rotation2d rotation) {
        double speed = anglePID.calculate(getAngle().getRotations(), rotation.getRotations());
        angleMotorSim.setInputVoltage(speed);
    }

    @Override
    public Encoder getEncoder() {
        return encoder;
    }

    @Override
    public void zeroWheels() {
        
    }

    @Override
    public void configure() {

    }

    @Override
    public void update(){
        angleMotorSim.update(0.02);
        driveMotorSim.update(0.02);
    }
    
}
