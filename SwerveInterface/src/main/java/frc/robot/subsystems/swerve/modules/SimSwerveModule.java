package frc.robot.subsystems.swerve.modules;


import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.DutyCycleEncoderSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
    private PhoenixPIDController anglePIDAlt;
    private PhoenixPIDController drivePIDAlt;

    private double numRotations = 0;

    public SimSwerveModule(SwerveModuleConstants constants){
        driveMotorOutput = constants.driveMotor.dcMotor;
        angleMotorOutput = constants.angleMotor.dcMotor;

        driveMotorSim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(driveMotorOutput, 0.0157, 1/constants.driveMotor.gearRatio), 
            driveMotorOutput);
        angleMotorSim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(angleMotorOutput, 0.001, constants.angleMotor.gearRatio), 
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

        anglePIDAlt = new PhoenixPIDController(anglePID.getP(), anglePID.getI(), anglePID.getD());
        drivePIDAlt = new PhoenixPIDController(drivePID.getP(), drivePID.getI(), drivePID.getD());
        anglePID.enableContinuousInput(-0.5, 0.5);
        drivePID.setTolerance(0.01);
        //drivePID.enableContinuousInput(-0.38, 0.38);
    }

    @Override
    public void setOpenDriveState(SwerveModuleState state) {
        
    }

    @Override
    public void setClosedDriveState(SwerveModuleState state) {
        SmartDashboard.putNumber("drive amps output", drivePID.calculate(getDriveVelocity(), state.speedMetersPerSecond));
        SmartDashboard.putNumber("setMPS", state.speedMetersPerSecond);
        SmartDashboard.putBoolean("isAtSetpoint", drivePID.atSetpoint());
        driveMotorSim.setInputVoltage(drivePID.calculate(getDriveVelocity(), state.speedMetersPerSecond));
        //driveMotorSim.setInputVoltage(drivePIDAlt.calculate(getDriveVelocity(), state.speedMetersPerSecond, Timer.getFPGATimestamp()));
    }

    @Override
    public double getDriveVelocity() {
        //System.out.println("getting drive velocity");
        SmartDashboard.putNumber("driveVel", driveMotorSim.getAngularVelocityRadPerSec());
        return (driveMotorSim.getAngularVelocityRadPerSec() * SwerveConstants.wheelCircumference);
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
        //setpoint = 0.6;
        anglePID.setSetpoint(setpoint);
        speed = anglePID.calculate(getAngle().getRotations());
        //speed = anglePIDAlt.calculate(getAngle().getRotations(), setpoint, Timer.getFPGATimestamp());
        SmartDashboard.putNumber("module angle", getAngle().getDegrees());
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
