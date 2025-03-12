package frc.robot.subsystems;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.studica.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.pathfinding.Pathfinding;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.Camera;
import frc.robot.utils.SwerveModule;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;


public class Swerve extends SubsystemBase {
	private final SwerveModule[] modules;
	private final XboxController xbox;
	public final SwerveDrivePoseEstimator swerveOdometry;

	private RobotConfig config; 

	private GenericEntry poseX = Shuffleboard.getTab("Odometry").add("Pose X", 0).withWidget("Text View").getEntry();
	private GenericEntry poseY = Shuffleboard.getTab("Odometry").add("Pose Y", 0).withWidget("Text View").getEntry();
	private GenericEntry poseRot = Shuffleboard.getTab("Odometry").add("Pose Rot", 0).withWidget("Text View").getEntry();
	
	private PIDController rotVisionPID = new PIDController(0.0025, 0, 0);
	private PIDController translationVisionPID = new PIDController(0.2, 0.0001, 0.0035);
	private PIDController gyroPID = new PIDController(0.1, 0, 0);

	private final AHRS gyro;
	private final Vision vision;

	public SwerveState state = SwerveState.DefaultState;
	private double translateX;
	private double translateY;
	private double rotationZ;

	private double xSetpoint = 0;
	private double ySetpoint = 0;
	private double rotationSetpoint = 0;

	private double maxSpeed = 0.5;
	private double maxAngularVelocity = 0.5;
	private double difference = 0;

	private boolean isFieldRelative = true;
	private boolean useVisionPose = true;

		/**
		 * The SwerveState of robot, which should be determined on field position & the beam break.
		 * States include: "DefaultState", "Intaking", "Scoring", "RobotRelative", "VisionGyro", "Vision", "LowerSpeed", "leftAlignment", "rightAlignment", & "centerAlignment".
		 */
		public enum SwerveState{
		DefaultState,
		RobotRelative,
		LowerSpeed,
		Vision,
		VisionWithGyro,
		Custom,
		RotationSetpoint,
		}

	/**
	 * Instantiates the swerve subsystem & sets up the modules and pose estimator.
	 * @param gyro - The NavX gyroscope
	 * @param vision - The vision subsystem, set up in RobotContainer
	 */
	public Swerve(AHRS gyro, Vision vision) {
		this.gyro = gyro;
		this.vision = vision;
		xbox = new XboxController(0);
		zeroGyro();
		
		modules = new SwerveModule[] {
			new SwerveModule(0, Constants.kSwerve.MOD_0_Constants),
			new SwerveModule(1, Constants.kSwerve.MOD_1_Constants),
			new SwerveModule(2, Constants.kSwerve.MOD_2_Constants),
			new SwerveModule(3, Constants.kSwerve.MOD_3_Constants),
		};
		swerveOdometry = new SwerveDrivePoseEstimator(Constants.kSwerve.KINEMATICS, getYaw(), getPositions(),vision.robotPose);

		for(SwerveModule mod : modules){
			Shuffleboard.getTab("DeviceOutputs").addDouble("Angle motor " + mod.moduleNumber + " current", () -> mod.angleMotor.getStatorCurrent().getValueAsDouble()).withWidget("Text View");
			Shuffleboard.getTab("DeviceOutputs").addDouble("Angle motor " + mod.moduleNumber + " velocity", () -> mod.angleMotor.getVelocity().getValueAsDouble()).withWidget("Text View");
			Shuffleboard.getTab("DeviceOutputs").addDouble("Angle motor " + mod.moduleNumber + " supply voltage", () -> mod.angleMotor.getSupplyVoltage().getValueAsDouble()).withWidget("Text View");
			Shuffleboard.getTab("DeviceOutputs").addDouble("Angle motor " + mod.moduleNumber + " output voltage", () -> mod.angleMotor.getMotorVoltage().getValueAsDouble()).withWidget("Text View");

			Shuffleboard.getTab("DeviceOutputs").addDouble("Drive motor " + mod.moduleNumber + " current", () -> mod.driveMotor.getStatorCurrent().getValueAsDouble()).withWidget("Text View");
			Shuffleboard.getTab("DeviceOutputs").addDouble("Drive motor " + mod.moduleNumber + " velocity", () -> mod.driveMotor.getVelocity().getValueAsDouble()).withWidget("Text View");
			Shuffleboard.getTab("DeviceOutputs").addDouble("Drive motor " + mod.moduleNumber + " supply voltage", () -> mod.driveMotor.getSupplyVoltage().getValueAsDouble()).withWidget("Text View");
			Shuffleboard.getTab("DeviceOutputs").addDouble("Drive motor " + mod.moduleNumber + " output voltage", () -> mod.driveMotor.getMotorVoltage().getValueAsDouble()).withWidget("Text View");
			Shuffleboard.getTab("DeviceOutputs").addDouble("Drive motor " + mod.moduleNumber + " temp C", () -> mod.driveMotor.getProcessorTemp().getValueAsDouble()).withWidget("Text View");

			Shuffleboard.getTab("test").add("CAN Coder " + mod.moduleNumber + " rotations", mod.getCanCoder());
		}
		
		AutoBuilder.configure(
				this::getPose, // Robot pose supplier
				this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
				this::getRelSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
				this::driveRobotRelative,
				Constants.kAuto.cont,
				Constants.kAuto.config,
				() -> {
				if(DriverStation.getAlliance().isPresent()){
					return DriverStation.getAlliance().get() == DriverStation.Alliance.Red;
				}else{
					return false;
				}
				},
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
	public Command drive(DoubleSupplier forwardBackAxis, DoubleSupplier leftRightAxis, DoubleSupplier rotationAxis, BooleanSupplier isFieldRelative, boolean isOpenLoop) {
		return run(() -> {
		// Grabbing input from suppliers.
		double forwardBack = forwardBackAxis.getAsDouble();
		double leftRight = leftRightAxis.getAsDouble();
		double rotation = rotationAxis.getAsDouble();

		// Adding deadzone.
		forwardBack = Math.abs(forwardBack) < Constants.kControls.AXIS_DEADZONE ? 0 : forwardBack;
		leftRight = Math.abs(leftRight) < Constants.kControls.AXIS_DEADZONE ? 0 : leftRight;
		rotation = Math.abs(rotation) < Constants.kControls.AXIS_DEADZONE ? 0 : rotation;
		
		// Converting to m/s
		forwardBack *= Constants.kSwerve.MAX_VELOCITY_METERS_PER_SECOND;
		leftRight *= Constants.kSwerve.MAX_VELOCITY_METERS_PER_SECOND;
		rotation *= Constants.kSwerve.MAX_ANGULAR_RADIANS_PER_SECOND;

		// Get desired module states
		ChassisSpeeds chassisSpeeds = isFieldRelative.getAsBoolean()
			? ChassisSpeeds.fromFieldRelativeSpeeds(forwardBack, leftRight, rotation, getYaw().unaryMinus())
			: new ChassisSpeeds(forwardBack, leftRight, rotation);

		SwerveModuleState[] states = Constants.kSwerve.KINEMATICS.toSwerveModuleStates(chassisSpeeds);

		setModuleStates(states, isOpenLoop);
		}).withName("SwerveDriveCommand");
	}

	/**
	 * @return The X Translation based on the SwerveState of the Robot.
	 * @see SwerveState
	 */
	public double getTransX(){
		return translateX;
	}
	/**
	 * @return The Y Translation based on the SwerveState of the Robot.
	 * @see SwerveState
	 */
	public double getTransY(){
		return translateY;
	}
	/**
	 * @return The Z Rotation based on the SwerveState of the Robot.
	 * @see SwerveState
	 */
	public double getRotationZ(){
		return rotationZ;
	}
	/**
	 * @return Whether FieldRelative is true or false, based on the SwerveState of the Robot.
	 * @see SwerveState
	 */
	public boolean getFieldRelative(){
		return isFieldRelative;
	}
	/**Sets the current SwerveState of the robot.
	 * @param  State - What state should be selected.
	 * @see SwerveState
	 */
	public void setState(SwerveState State){
		this.state = State;
	}

	/**Decides the Robot's movement options based on the current SwerveState.
	 * @see SwerveState
	 */
	private void handleStates(){
		switch (state) {
		case DefaultState:
		case LowerSpeed:
			translateX = Constants.kControls.X_DRIVE_LIMITER.calculate(Math.pow(xbox.getLeftX(), 3 / 1) * maxSpeed);
			translateY = Constants.kControls.Y_DRIVE_LIMITER.calculate(Math.pow(xbox.getLeftY(), 3 / 1) * maxSpeed);
			rotationZ = Constants.kControls.THETA_DRIVE_LIMITER.calculate(Math.pow(xbox.getRightX(), 3 / 1) * maxSpeed);
			isFieldRelative = true;
			break;

		case RobotRelative:
			translateX = Constants.kControls.X_DRIVE_LIMITER.calculate(Math.pow(xbox.getLeftX(), 3 / 1) * maxSpeed);
			translateY = Constants.kControls.Y_DRIVE_LIMITER.calculate(Math.pow(xbox.getLeftY(), 3 / 1) * maxSpeed);
			rotationZ = Constants.kControls.THETA_DRIVE_LIMITER.calculate(Math.pow(xbox.getRightX(), 3 / 1) * maxSpeed);
			isFieldRelative = false;
			break;

		case Vision:
			double rotMeasure = 0;
			if(vision.frontCamera.getBestTarget() != null){
				rotMeasure = vision.getRobotToTarget(vision.frontCamera.getTarget(vision.getWantedTarget()).getFiducialId()).getRotation().toRotation2d().getDegrees();
				if(rotMeasure < 0){
					rotMeasure += 360;
				}
			}
			translateX = vision.frontCamera.getBestTarget() != null 
				? translationVisionPID.calculate(vision.getRobotToTarget(vision.frontCamera.getBestTarget().getFiducialId()).getX(), vision.getTagAlignment().getX())
				: 0;
			translateY = vision.frontCamera.getBestTarget() != null
				? -translationVisionPID.calculate(vision.getRobotToTarget(vision.frontCamera.getBestTarget().getFiducialId()).getY(), vision.getTagAlignment().getY())
				: 0;
			rotationZ = rotVisionPID.calculate(rotMeasure, vision.getTagAlignment().getRotation().getDegrees());
			isFieldRelative = false;
			break;

		case VisionWithGyro:
			translateX = vision.frontCamera.getBestTarget() != null 
				? translationVisionPID.calculate(vision.getRobotToTarget(vision.frontCamera.getBestTarget().getFiducialId()).getX(), vision.getTagAlignment().getX())
				: 0;
			translateY = vision.frontCamera.getBestTarget() != null 
				? -translationVisionPID.calculate(vision.getRobotToTarget(vision.frontCamera.getBestTarget().getFiducialId()).getY(), vision.getTagAlignment().getY())
				: 0;
			rotationZ = gyroPID.calculate(gyro.getYaw(), rotationSetpoint);
			rotationZ = 0;
			isFieldRelative = false;
			break;

		case RotationSetpoint:
			translateX = Constants.kControls.X_DRIVE_LIMITER.calculate(Math.pow(xbox.getLeftX(), 3 / 1) * maxSpeed);
			translateY = Constants.kControls.Y_DRIVE_LIMITER.calculate(Math.pow(xbox.getLeftY(), 3 / 1) * maxSpeed);
			rotationZ = gyroPID.calculate(gyro.getYaw(), rotationSetpoint);
			isFieldRelative = true;
			break;

		case Custom:
		default:
			break;

		/*case Vision:
			if (vision.frontCamera.canSeeTarget(18)){
			var rotMeasure = vision.getRobotToTarget(18).getRotation().toRotation2d().getDegrees();
			if(rotMeasure < 0){
				rotMeasure += 360;
			}
			translateX = translationVisionPID.calculate(vision.getRobotToTarget(18).getX(), 0);
			translateY = translationVisionPID.calculate(vision.getRobotToTarget(18).getY(), 0);
			rotationZ = rotVisionPID.calculate(rotMeasure, 180);
			} else {
			translateX = 0;
			translateY = 0;
			rotationZ = 0;
			}
			isFieldRelative = false;
			break;*/
		}
	}

	/** To be used by auto. Use the drive method during teleop. */
	public void setModuleStates(SwerveModuleState[] states) {
		setModuleStates(states, false);
	}
	/**Sets the state for each given module, and desaturates the wheels.
	 * 
	 * @param states - What state to set the modules to
	 * @param isOpenLoop - true or false boolean
	 */
	private void setModuleStates(SwerveModuleState[] states, boolean isOpenLoop) {
		// Makes sure the module states don't exceed the max speed.
		SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.kSwerve.MAX_VELOCITY_METERS_PER_SECOND);

		for (int i = 0; i < modules.length; i++) {
			modules[i].setState(states[modules[i].moduleNumber], isOpenLoop);
		}
	}
	/** 
	 * @return The list of each module's current state.
	 */
	public SwerveModuleState[] getStates() {
		SwerveModuleState currentStates[] = new SwerveModuleState[modules.length];
		for (int i = 0; i < modules.length; i++) {
			currentStates[i] = modules[i].getState();
		}

		return currentStates;
	}
	/** 
	 * @return The list of each module's current state but inverted, in order to keep things CCW+.
	 */
	public SwerveModuleState[] getInvertedStates() {
		SwerveModuleState currentStates[] = new SwerveModuleState[modules.length];
		for (int i = 0; i < modules.length; i++) {
			currentStates[i] = modules[i].getInvertedState();
		}

		return currentStates;
	}
	
	/**
	 * @return The list of each module's current position.
	 */
	public SwerveModulePosition[] getPositions() {
		SwerveModulePosition currentStates[] = new SwerveModulePosition[modules.length];
		for (int i = 0; i < modules.length; i++) {
			currentStates[i] = modules[i].getPosition();
		}
		return currentStates;
	}
	/**
	 * @return The list of each module's current position, but inverted, in order to keep things CCW+.
	 */
	public SwerveModulePosition[] getInvertedPositions() {
		SwerveModulePosition currentStates[] = new SwerveModulePosition[modules.length];
		for (int i = 0; i < modules.length; i++) {
			currentStates[i] = modules[i].getInvertedPosition();
		}
		return currentStates;
	}

	/**
	 * @return The current Yaw from the gyroscope in degrees.
	 */
	public Rotation2d getYaw() {
		return Rotation2d.fromDegrees(-gyro.getYaw());
	}
	/**
	 * @return The current Angle from the gyroscope in degrees, goes to 360 instead of wrapping to 180 like {@link #getYaw}.
	 */
	public Rotation2d getAngle() {
		return Rotation2d.fromDegrees(-gyro.getAngle());
	}

	/**
	 *@return Runs {@link #zeroGyro} as a Command.
	*/
	public Command zeroGyroCommand() {
		return runOnce(this::zeroGyro).withName("ZeroGyroCommand");
	}
	/**
	 * Zeros the gyroscope's Yaw (Robot's rotation from left to right)
	 */
	private void zeroGyro() {
		gyro.zeroYaw();
		gyro.setAngleAdjustment(180);
	}

	/**
	 * @return The current Estimated Position of the swerveOdometry
	 */
	public Pose2d getPose() {
		var swervePose = swerveOdometry.getEstimatedPosition();
		return swervePose;
	}

	/**
	 * Resets the swerveOdometry's Pose to a chosen position
	 * @param pose The chosen Position
	 */
	public void resetOdometry(Pose2d pose) { 
		swerveOdometry.resetPose(pose);
	}
	/**
	 * @return Runs {@link #resetOdometry} as a Command.
	 */
	public Command resetOdometryCommand() {
		return new InstantCommand(() -> resetOdometry(new Pose2d(poseX.getDouble(0), poseY.getDouble(0), getYaw())));
	}

	/**
	 * 
	 * @return The Relative Speed of the Chassis, using the Inverted State, {@code ONLY} for the Pathplanner.
	 */
	public ChassisSpeeds getRelSpeeds() {
		ChassisSpeeds relSpeed = Constants.kSwerve.KINEMATICS.toChassisSpeeds(getInvertedStates());
		return relSpeed;
	}
	/**
	 * Corrects Translational skew, and sets SwerveModule's speed.
	 * @param speeds The Speed to set the SwerveModules to.
	 * @see #setModuleStates
	 */
	public void driveRobotRelative(ChassisSpeeds speeds){
		speeds = ChassisSpeeds.discretize(speeds, 0.02);
		speeds.omegaRadiansPerSecond *= -1;
		speeds.vyMetersPerSecond *= -1;
		SwerveModuleState[] states = Constants.kSwerve.KINEMATICS.toSwerveModuleStates(speeds);
		setModuleStates(states);
	}

	/**
	 * Periodic command called 50 times per second.
	 * <p>
	 * Updates the swerve odometry using update, {@link #getAngle()}, and {@link #getInvertedPositions()}.
	 * Also updates the cameras. Uses addVisionMeasurement, getRobotPose, getFPGATimestamp, getPoseAmbiguity methods.
	 * <p>
	 * Runs the {@link #handleStates()} command.
	 * @see -Link to update: https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/math/estimator/PoseEstimator.html#update(edu.wpi.first.math.geometry.Rotation2d,T).
	 * @see -Link to addVisionMeasurement: https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/math/estimator/PoseEstimator.html#addVisionMeasurement(edu.wpi.first.math.geometry.Pose2d,double,edu.wpi.first.math.Matrix).
	 * @see -getRobotPose is found under: frc.robot.utils.Camera.
	 * @see -Link to getFPGATimestamp: https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/wpilibj/Timer.html#getFPGATimestamp().
	 * @see -getPoseAmbiguity is found under: frc.robot.utils.Camera.
	 */
	@Override 
	public void periodic() {
		swerveOdometry.update(getAngle(), getInvertedPositions());
		if(useVisionPose){
			for(Camera camera : vision.cameraList){
				if(camera.updatePose()){
					swerveOdometry.addVisionMeasurement(camera.getRobotPose(), Timer.getFPGATimestamp(), camera.getPoseAmbiguity());
				}
			}
		}	

		//threshold
		if(difference <= 0.15){
			difference = 0;
		}else if(difference >= 0.9){
			difference = 0.9;
		}
		//determines the max speed from 100% - x%
		maxSpeed = 1 - difference;
		maxAngularVelocity = 1 - (difference);
		

		handleStates();
	}

	@Override
	public void initSendable(SendableBuilder builder) {
		super.initSendable(builder);
		for (SwerveModule module : modules) {
			builder.addStringProperty(
				String.format("Module %d", module.moduleNumber),
				() -> {
					SwerveModuleState state = module.getState();
					return String.format("%6.2fm/s %6.3fdeg", state.speedMetersPerSecond, state.angle.getDegrees());
				},
				null);

				builder.addDoubleProperty(
					String.format("Cancoder %d", module.moduleNumber),
					() -> module.getCanCoder(),
				null);

				
				builder.addDoubleProperty(
					String.format("Angle %d", module.moduleNumber),
					() -> module.getAngle().getDegrees(),
				null);
		}
	}

	/**
	 * @return The NavX Gyroscope according to {@link Swerve}.
	 */
	public AHRS getGyro(){
		return gyro;
	}

	public PIDController getRotationPID(){
		return gyroPID;
	}

	/**
	 * Subtracts the swerve speed by this value, in a percentage of 0 (0 percent) to 1 (100 percent).
	 * @param value -Type "double", value of 0 to 1, being 0 is not slowing swerve at all, and 1 being swerve modules move at 0% speed. (There is safty so it is never 0% speed)
	 */
	public void setSubtractedSpeed(double value){
		difference = value;
	}

	public void setRotationSetpoint(double setpoint){
		rotationSetpoint = setpoint;
	}
	public void setUseVisionPose(boolean bool){
		useVisionPose = bool;
	}
}
