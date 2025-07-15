package frc.robot.subsystems.swerve.modules;


import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.DutyCycleEncoderSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.swerve.config.DrivebaseConfig;
import frc.robot.subsystems.swerve.config.SwerveModuleConfig;
import frc.robot.utils.encoder.GenericEncoder;

public class SimSwerveModule implements GenericSwerveModule{
    private DCMotor driveMotorOutput;
    private DCMotor angleMotorOutput;
    private DCMotorSim driveMotorSim;
    private DCMotorSim angleMotorSim;

    private GenericEncoder encoder;

    private double driveMotorAmps;
    private double angleMotorAmps;
    private PIDController drivePID;
    private PIDController anglePID;
    private SimpleMotorFeedforward driveFeedforward = new SimpleMotorFeedforward(0.01, 2.69, 0.19);

    public SimSwerveModule(SwerveModuleConfig constants){
        driveMotorOutput = constants.driveMotor.dcMotor;
        angleMotorOutput = constants.angleMotor.dcMotor;

        driveMotorSim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(driveMotorOutput, constants.driveMotor.momentOfInertia, constants.driveMotor.gearRatio), 
            driveMotorOutput);
        angleMotorSim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(angleMotorOutput, constants.angleMotor.momentOfInertia, constants.angleMotor.gearRatio), 
            angleMotorOutput);

        encoder = constants.encoder;

        driveMotorAmps = constants.driveMotor.currentLimit; //TODO is there some formula/estimation?
        angleMotorAmps = constants.angleMotor.currentLimit;
        // constructs a new PID controller for each object. 
        // this avoids the PID from doing too much and trying to compensate for it by causing unwanted behavior
        anglePID = constants.angleMotor.pid;
        anglePID = new PIDController(anglePID.getP(), anglePID.getI(), anglePID.getD());
        drivePID = constants.angleMotor.pid;
        drivePID = new PIDController(drivePID.getP(), drivePID.getI(), drivePID.getD());

        anglePID.enableContinuousInput(-0.5, 0.5);
    }

    @Override
    public void setOpenDriveState(SwerveModuleState state) {
        
    }

    @Override
    public void setClosedDriveState(SwerveModuleState state) {
        SmartDashboard.putNumber("setMPS", state.speedMetersPerSecond);
        SmartDashboard.putBoolean("isAtSetpoint", drivePID.atSetpoint());
        double ffCalc = driveFeedforward.calculate(state.speedMetersPerSecond);
        double input = drivePID.calculate(getDriveVelocity(), state.speedMetersPerSecond);
        //input = drivePID.calculate(getDriveVelocity(), 100 * SwerveConstants.wheelCircumference);

        driveMotorSim.setInputVoltage(input);
    }

    @Override
    public double getDriveVelocity() {
        SmartDashboard.putNumber("driveVel", ((driveMotorSim.getAngularVelocityRadPerSec() / (2*Math.PI)) * SwerveConstants.wheelCircumference));
        SmartDashboard.putNumber("driveVoltage", driveMotorSim.getInputVoltage());
        return ((driveMotorSim.getAngularVelocityRadPerSec() / (2*Math.PI)) * SwerveConstants.wheelCircumference);
    }

    @Override
    public double getDrivePosition() {
        return driveMotorSim.getAngularPositionRotations();
    }

    @Override
    public Rotation2d getAngle() {
        double rotations = angleMotorSim.getAngularPositionRotations();
        SmartDashboard.putNumber("rotations", rotations);
        return Rotation2d.fromRotations(rotations);
    }

    @Override
    public void setAngle(Rotation2d rotation) {
        double speed;
        double setpoint = rotation.getRotations();
        anglePID.setSetpoint(setpoint);
        speed = anglePID.calculate(getAngle().getRotations());
        SmartDashboard.putNumber("module angle", getAngle().getDegrees());
        angleMotorSim.setInputVoltage(speed);
    }

    @Override
    public GenericEncoder getEncoder() {
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
