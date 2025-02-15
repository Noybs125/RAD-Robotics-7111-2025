package frc.robot.subsystems;

import java.lang.reflect.Field;
import java.security.PublicKey;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.studica.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
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
import frc.robot.subsystems.Vision.VisionState;
import frc.robot.utils.Camera;
import frc.robot.utils.SwerveModule;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.ComplexWidget;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;


public class Swerve extends SubsystemBase {
  private final SwerveModule[] modules;
  public final XboxController xbox;
  public final SwerveDrivePoseEstimator swerveOdometry;

  private Field2d field = new Field2d();
  private FieldObject2d fieldObjectPose = field.getObject("FieldPosition");
  public RobotConfig config;

  private GenericEntry poseX = Shuffleboard.getTab("Odometry").add("Pose X", 0).withWidget("Text View").getEntry();
  private GenericEntry poseY = Shuffleboard.getTab("Odometry").add("Pose Y", 0).withWidget("Text View").getEntry();
  private GenericEntry poseRot = Shuffleboard.getTab("Odometry").add("Pose Rot", 0).withWidget("Text View").getEntry();
  
  private PIDController rotVisionPID = new PIDController(0.0025, 0,0);
  private PIDController translationVisionPID = new PIDController(0.2, 0.0001,0.0035);
  private PIDController gyroPID;

  private final AHRS gyro;
  private final Vision vision;
  private ComplexWidget fieldPublisher;
  private ComplexWidget field2Publisher;

  public SwerveState state = SwerveState.DefaultState;
  private double translateX;
  private double translateY;
  private double rotationZ;

  private boolean isFieldRelative = true;

  public Swerve(AHRS gyro, Vision vision) {
    this.gyro = gyro;
    this.vision = vision;
    fieldPublisher = Shuffleboard.getTab("Odometry").add("field odometry", field).withWidget("Field");
    xbox = new XboxController(2);
    zeroGyro();
    

    modules = new SwerveModule[] {
      new SwerveModule(0, Constants.kSwerve.MOD_0_Constants),
      new SwerveModule(1, Constants.kSwerve.MOD_1_Constants),
      new SwerveModule(2, Constants.kSwerve.MOD_2_Constants),
      new SwerveModule(3, Constants.kSwerve.MOD_3_Constants),
    };
    swerveOdometry = new SwerveDrivePoseEstimator(Constants.kSwerve.KINEMATICS, getYaw(), getPositions(),vision.robotPose);

    /*try{
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      
      e.printStackTrace();
    }*/
    
    AutoBuilder.configure(
            this::getPose, // Robot pose supplier
            this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getRelSpeedsNonSuplier, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
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

  public enum SwerveState{
    DefaultState,
    Intaking,
    Scoring,
    RobotRelative,
    VisionGyro,
    Vision
  }

  public double getTransX(){
    return translateX;
  }
  public double getTransY(){
    return translateY;
  }
  public double getRotationZ(){
    return rotationZ;
  }
  public boolean getFieldRelative(){
    return isFieldRelative;
  }
  public void setState(SwerveState State){
    this.state = State;
  }

  private void handleStates()
  {
    switch (state) {
      case DefaultState:
        translateX = Constants.kControls.X_DRIVE_LIMITER.calculate(Math.pow(xbox.getLeftX(), 3 / 1));
        translateY = Constants.kControls.Y_DRIVE_LIMITER.calculate(Math.pow(xbox.getLeftY(), 3 / 1));
        rotationZ = Constants.kControls.THETA_DRIVE_LIMITER.calculate(Math.pow(xbox.getRightX(), 3 / 1));
        isFieldRelative = true;
        break;

      case Intaking:
        translateX = 0;
        translateY = 0;
        rotationZ = 0;
        isFieldRelative = true;
        break;

      case Scoring:
        translateX = 0;
        translateY = 0;
        rotationZ = 0;
        isFieldRelative = true;
        break;

      case RobotRelative:
        translateX = 0;
        translateY = 0;
        rotationZ = 0;
        isFieldRelative = true;
        break;

      case VisionGyro:
        translateX = 0;
        translateY = 0;
        rotationZ = 0;
        isFieldRelative = true;
        break;

      case Vision:
        if (vision.canSeeTarget(18, vision.orangepi1)){
          var rotMeasure = vision.getAlignmentToTarget(18, vision.orangepi1).getRotation().getDegrees();
          if(rotMeasure < 0){
            rotMeasure += 360;
          }
          translateX = translationVisionPID.calculate(vision.getAlignmentToTarget(18, vision.orangepi1).getX(), 0);
          translateY = translationVisionPID.calculate(vision.getAlignmentToTarget(18, vision.orangepi1).getY(),0.35);
          rotationZ = rotVisionPID.calculate(rotMeasure, 180);
        } else {
          translateX = 0;
          translateY = 0;
          rotationZ = 0;
        }
        isFieldRelative = false;
        break;
    }
  }
  /** To be used by auto. Use the drive method during teleop. */
  public void setModuleStates(SwerveModuleState[] states) {
    setModuleStates(states, false);
  }

  private void setModuleStates(SwerveModuleState[] states, boolean isOpenLoop) {
    // Makes sure the module states don't exceed the max speed.
    SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.kSwerve.MAX_VELOCITY_METERS_PER_SECOND);

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

  public SwerveModuleState[] getInvertedStates() {
    SwerveModuleState currentStates[] = new SwerveModuleState[modules.length];
    for (int i = 0; i < modules.length; i++) {
      currentStates[i] = modules[i].getInvertedState();
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
  public SwerveModulePosition[] getInvertedPositions() {
    SwerveModulePosition currentStates[] = new SwerveModulePosition[modules.length];
    for (int i = 0; i < modules.length; i++) {
      currentStates[i] = modules[i].getInvertedPosition();
    }
    return currentStates;
  }

  public Rotation2d getYaw() {
    return Rotation2d.fromDegrees(-gyro.getYaw());
  }
  public Rotation2d getAngle() {
    return Rotation2d.fromDegrees(-gyro.getAngle());
  }

  public Command zeroGyroCommand() {
    return runOnce(this::zeroGyro).withName("ZeroGyroCommand");
  }

  private void zeroGyro() {
    gyro.zeroYaw();
  }

  public Pose2d getPose() {
    var swervePose = swerveOdometry.getEstimatedPosition();
    return swervePose;
  }

  public void resetOdometry(Pose2d pose) { // not currently used, using addVisionMeasurements in periodic instead.
      swerveOdometry.resetPose(pose);
  }

  public Command resetOdometryCommand() {
    return new InstantCommand(() -> resetOdometry(new Pose2d(poseX.getDouble(0), poseY.getDouble(0), getYaw())));
  }

  public ChassisSpeeds getRelSpeedsNonSuplier() {
    ChassisSpeeds relSpeed = Constants.kSwerve.KINEMATICS.toChassisSpeeds(getInvertedStates());
    return relSpeed;
  }

  public void driveRobotRelative(ChassisSpeeds speeds){
    speeds = ChassisSpeeds.discretize(speeds, 0.02);
    speeds.omegaRadiansPerSecond *= -1;
    speeds.vyMetersPerSecond *= -1;
    SwerveModuleState[] states = Constants.kSwerve.KINEMATICS.toSwerveModuleStates(speeds);
    setModuleStates(states);

  }
  
  @Override 
  public void periodic() {
      swerveOdometry.update(getAngle(), getInvertedPositions());
    for(Camera camera : vision.cameraList){
      if(camera.updatePose()){
        swerveOdometry.addVisionMeasurement(camera.getRobotPose(), Timer.getFPGATimestamp(), camera.getPoseAmbiguity());
      }
    }
    if(vision.canSeeTarget(18, vision.orangepi1)){
      SmartDashboard.putNumber("Vison TranslateY", vision.getAlignmentToTarget(18, vision.orangepi1).getY());
      SmartDashboard.putNumber("Vison TranslateX", vision.getAlignmentToTarget(18, vision.orangepi1).getX());

    }
    
    for(SwerveModule mod : modules){
      SmartDashboard.putNumber("angle Motor " + mod +" current", mod.angleMotor.getStatorCurrent().getValueAsDouble());
      SmartDashboard.putNumber("drive motor " + mod + " current", mod.driveMotor.getStatorCurrent().getValueAsDouble());
    }
    
    handleStates();
    field.setRobotPose(getPose());
    fieldObjectPose.setPose(new Pose2d(poseX.getDouble(0), poseY.getDouble(0), Rotation2d.fromDegrees(poseRot.getDouble(0))));
    
    
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

  public AHRS getGyro(){
    return gyro;
  }
}
