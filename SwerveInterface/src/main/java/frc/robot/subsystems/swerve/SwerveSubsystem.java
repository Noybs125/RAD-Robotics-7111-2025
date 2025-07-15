package frc.robot.subsystems.swerve;

import java.util.function.DoubleSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.DeviceConfigs;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.swerve.swervegyro.RealSwerveGyro;
import frc.robot.subsystems.swerve.swervegyro.SimSwerveGyro;
import frc.robot.subsystems.swerve.swervegyro.GenericSwerveGyro;
import frc.robot.utils.gyro.NavXGyro;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.DriverStation.Alliance;


public class SwerveSubsystem extends SubsystemBase {
    private final SwerveModule[] modules;

    private final SwerveDriveOdometry swerveOdometry;
    private Field2d field = new Field2d();

    private final GenericSwerveGyro gyro;
    private SwerveModuleState[] states = new SwerveModuleState[]{};

    public PIDController translationXPID = new PIDController(1, 0, 0);
    public PIDController translationYPID = translationXPID;
    public PIDController rotationPID = new PIDController(1, 0, 0);

    private TrapezoidProfile.Constraints xConstraints = new TrapezoidProfile.Constraints(30, 30);
    public ProfiledPIDController profiledXPID = new ProfiledPIDController(1, 0, 0, xConstraints);

    private TrapezoidProfile.Constraints yConstraints = new TrapezoidProfile.Constraints(30, 30);
    public ProfiledPIDController profiledYPID = new ProfiledPIDController(1, 0, 0, yConstraints);

    private TrapezoidProfile.Constraints rotationConstraints = new TrapezoidProfile.Constraints(720, 720);
    public ProfiledPIDController profiledRotationPID = new ProfiledPIDController(1, 0, 0, rotationConstraints);

    private StructArrayPublisher<SwerveModuleState> commandedStatePublisher = NetworkTableInstance.getDefault().getStructArrayTopic("Commanded Swerve States", SwerveModuleState.struct).publish();
    private StructArrayPublisher<SwerveModuleState> actualStatePublisher = NetworkTableInstance.getDefault().getStructArrayTopic("Actual Swerve States", SwerveModuleState.struct).publish();

    public SwerveSubsystem() {
        modules = new SwerveModule[] {
            new SwerveModule(0, SwerveConstants.drivebaseConfig.moduleTypes[0]),
            new SwerveModule(1, SwerveConstants.drivebaseConfig.moduleTypes[1]),
            new SwerveModule(2, SwerveConstants.drivebaseConfig.moduleTypes[2]),
            new SwerveModule(3, SwerveConstants.drivebaseConfig.moduleTypes[3]),
        };

        gyro = RobotBase.isReal()
            ? new RealSwerveGyro(new NavXGyro())
            : new SimSwerveGyro(this::getStates, SwerveConstants.kinematics);
        gyro.setInverted(true);
        zeroGyro();

        translationXPID.setTolerance(0.05);
        translationYPID.setTolerance(0.05);
        rotationPID.setTolerance(1);
        rotationPID.enableContinuousInput(-180, 180);

        swerveOdometry = new SwerveDriveOdometry(SwerveConstants.kinematics, getYaw(), getPositions());
        
        RobotConfig config = null;
        try{
            config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            // Handle exception as needed
            e.printStackTrace();
        }

        AutoBuilder.configure(
            this::getPose, // Robot pose supplier
            this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getRelSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            this::setRelSpeeds, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
            new PPHolonomicDriveController( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                    new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                    new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
            ),
            config,
            () -> 
                DriverStation.getAlliance().isPresent()
                    ? DriverStation.getAlliance().get() == Alliance.Red
                    : false,
            this // Reference to this subsystem to set requirements
        );
    }

    /** 
     * This is called a command factory method, and these methods help reduce the
     * number of files in the command folder, increasing readability and reducing
     * boilerplate. 
     * 
     * Double suppliers are just any function that returns a double.
     */
    public Command drive(DoubleSupplier forwardBackAxis, DoubleSupplier leftRightAxis, DoubleSupplier rotationAxis, boolean isFieldRelative, boolean isOpenLoop) {
        return run(() -> {
            // Grabbing input from suppliers.
            double forwardBack = forwardBackAxis.getAsDouble();
            double leftRight = leftRightAxis.getAsDouble();
            double rotation = rotationAxis.getAsDouble();

            // Adding deadzone.
            forwardBack = Math.abs(forwardBack) < ControllerConstants.axisDeadzone ? 0 : forwardBack;
            leftRight = Math.abs(leftRight) < ControllerConstants.axisDeadzone ? 0 : leftRight;
            rotation = Math.abs(rotation) < ControllerConstants.axisDeadzone ? 0 : rotation;

            // Converting to m/s
            forwardBack *= SwerveConstants.maxDriveVelocity;
            leftRight *= SwerveConstants.maxDriveVelocity;
            rotation *= SwerveConstants.maxAngularVelocity;

            // Get desired module states.
            ChassisSpeeds chassisSpeeds = isFieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(forwardBack, leftRight, rotation, getYaw())
                : new ChassisSpeeds(forwardBack, leftRight, rotation);

            SwerveModuleState[] states = SwerveConstants.kinematics.toSwerveModuleStates(chassisSpeeds);

            setModuleStates(states, isOpenLoop);
        });
    }

    /** To be used by auto. Use the drive method during teleop. */
    public void setModuleStates(SwerveModuleState[] states) {
        setModuleStates(states, false);
    }

    private void setModuleStates(SwerveModuleState[] states, boolean isOpenLoop) {
        // Makes sure the module states don't exceed the max speed.
        SwerveDriveKinematics.desaturateWheelSpeeds(states, SwerveConstants.maxDriveVelocity);
        this.states = states;
        for (int i = 0; i < modules.length; i++) {
            modules[i].setState(states[modules[i].moduleNumber], isOpenLoop);
        }
    }

    public SwerveModuleState[] getStates() {
        SwerveModuleState currentStates[] = new SwerveModuleState[modules.length];
        for (int i = 0; i < modules.length; i++) {
            currentStates[i] = modules[i].getState();
        }
        return currentStates;
    }

    public SwerveModulePosition[] getPositions() {
        SwerveModulePosition currentStates[] = new SwerveModulePosition[modules.length];
        for (int i = 0; i < modules.length; i++) {
            currentStates[i] = modules[i].getPosition();
        }

        return currentStates;
    }

    public Rotation2d getYaw() {
        return gyro.getRotation(); //Rotation2d.fromDegrees(gyro.getYaw());
    }

    public Command zeroGyroCommand() {
        return runOnce(this::zeroGyro).withName("ZeroGyroCommand");
    }

    private void zeroGyro() {
        gyro.setRotation(Rotation2d.kZero);
    }

    public Pose2d getPose() {
        return swerveOdometry.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        swerveOdometry.resetPosition(getYaw(), getPositions(), pose);
    }

    public ChassisSpeeds getRelSpeeds() {
        ChassisSpeeds relSpeed = SwerveConstants.kinematics.toChassisSpeeds(getStates());
        return relSpeed;
    }

    public void setRelSpeeds(ChassisSpeeds speeds){
        speeds.omegaRadiansPerSecond = -speeds.omegaRadiansPerSecond;
        speeds = ChassisSpeeds.discretize(speeds, 0.02);
        SwerveModuleState[] states = SwerveConstants.kinematics.toSwerveModuleStates(speeds);
        setModuleStates(states);
    }
    
    @Override 
    public void periodic() {
        gyro.update();
        swerveOdometry.update(getYaw(), getPositions());
        commandedStatePublisher.set(states);

        for(SwerveModule mod : modules){
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Cancoder", mod.getEncoder().getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Integrated", mod.getPosition().angle.getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);
        }
        SmartDashboard.putNumber("Gyro Yaw", getYaw().getDegrees());

        field.setRobotPose(getPose());
        SmartDashboard.putData(field);

        actualStatePublisher.set(getStates());
    }

    public void simulationPeriodic(){
        for (SwerveModule mod : modules) {
            mod.module.update();
        }
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        for (SwerveModule module : modules) {
            builder.addDoubleProperty(
            String.format("Drive Pos %d", module.moduleNumber),
            () -> module.getPosition().distanceMeters,
            null);

            
            builder.addDoubleProperty(
            String.format("Angle %d", module.moduleNumber),
            () -> module.getAngle().getDegrees(),
            null);
        }
    }
}
