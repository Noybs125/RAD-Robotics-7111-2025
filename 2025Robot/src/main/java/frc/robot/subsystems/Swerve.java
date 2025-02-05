package frc.robot.subsystems;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.studica.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.Camera;
import frc.robot.utils.SwerveModule;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.ComplexWidget;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;


public class Swerve extends SubsystemBase {
  private final SwerveModule[] modules;

  public final SwerveDrivePoseEstimator swerveOdometry;

  private SwerveDriveOdometry odometry2;
  private Field2d field = new Field2d();
  private Field2d fieldcam = new Field2d();
  private Field2d fieldWheel = new Field2d();
  public RobotConfig config;

  private PIDController visionPID = new PIDController(0.1, 0,0);
  private PIDController gyroPID;

  private final AHRS gyro;
  private final Vision vision;
  private ComplexWidget fieldPublisher;
  private ComplexWidget field2Publisher;

  public SwerveState state;
  private double translateX;
  private double translateY;
  private double rotationZ;

  private boolean isFieldRelative = true;

  public Swerve(AHRS gyro, Vision vision) {
    this.gyro = gyro;
    this.vision = vision;
    fieldPublisher = Shuffleboard.getTab("Odometry").add("field odometry", field).withWidget("Field");
    fieldPublisher = Shuffleboard.getTab("April Tag Odometry").add("april tag field odometry", fieldcam).withWidget("April Fields");
    fieldPublisher = Shuffleboard.getTab("Wheel Odometry").add("Wheel Odometry Field", fieldWheel).withWidget("Wheel Field");
    zeroGyro();
    

    modules = new SwerveModule[] {
      new SwerveModule(0, Constants.kSwerve.MOD_0_Constants),
      new SwerveModule(1, Constants.kSwerve.MOD_1_Constants),
      new SwerveModule(2, Constants.kSwerve.MOD_2_Constants),
      new SwerveModule(3, Constants.kSwerve.MOD_3_Constants),
    };
    odometry2 = new SwerveDriveOdometry(Constants.kSwerve.KINEMATICS, getYaw().unaryMinus(), getPositions());
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
  public void setState(SwerveState State){
    this.state = State;
  }

  private void handleStates()
  {
    switch (state) {
      case DefaultState:
        translateX = 0;
        translateY = 0;
        rotationZ = 0;
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
        translateX = 0;
        translateY = 0;
        rotationZ = 0;
        isFieldRelative = true;
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

  public SwerveModulePosition[] getPositions() {
    SwerveModulePosition currentStates[] = new SwerveModulePosition[modules.length];
    for (int i = 0; i < modules.length; i++) {
      currentStates[i] = modules[i].getPosition();
    }

    return currentStates;
  }

  public Rotation2d getYaw() {
    return Rotation2d.fromDegrees(-gyro.getYaw());
  }
  public Rotation2d getAngle() {
    return Rotation2d.fromDegrees(gyro.getAngle());
  }

  public Command zeroGyroCommand() {
    return runOnce(this::zeroGyro).withName("ZeroGyroCommand");
  }

  private void zeroGyro() {
    gyro.zeroYaw();
  }

  public Pose2d getPose() {
    return new Pose2d(swerveOdometry.getEstimatedPosition().getX(), -swerveOdometry.getEstimatedPosition().getY(), swerveOdometry.getEstimatedPosition().getRotation());
  }

  public void resetOdometry(Pose2d pose) { // not currently used, using addVisionMeasurements in periodic instead.
      swerveOdometry.resetPose(pose);
  }

  public Command resetOdometryCommand() {
    return new InstantCommand(() -> resetOdometry(new Pose2d(0, 0, getYaw())));
  }

  public ChassisSpeeds getRelSpeedsNonSuplier() {
    ChassisSpeeds relSpeed = Constants.kSwerve.KINEMATICS.toChassisSpeeds(getStates());
    return relSpeed;
  }

  public void driveRobotRelative(ChassisSpeeds speeds){
    speeds = ChassisSpeeds.discretize(speeds, 0.02);
    SwerveModuleState[] states = Constants.kSwerve.KINEMATICS.toSwerveModuleStates(speeds);
    setModuleStates(states);

  }
  
  @Override 
  public void periodic() {
      odometry2.update(getYaw().unaryMinus(), getPositions());
      swerveOdometry.update(getAngle(), getPositions());
    for(Camera camera : vision.cameraList){
      if(camera.updatePose()){
        swerveOdometry.addVisionMeasurement(camera.getRobotPose(), Timer.getFPGATimestamp(), camera.getPoseAmbiguity());
        fieldcam.setRobotPose(camera.getRobotPose());
      }
    }
    
    for(SwerveModule mod : modules){
      SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Cancoder", mod.getCanCoderDegrees().getDegrees());
      SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Integrated", mod.getPosition().angle.getDegrees());
      SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);
      SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Angle Position", mod.getAngle().getRadians());    
    }
    SmartDashboard.putNumber("Gyro", getYaw().getDegrees());
    SmartDashboard.putNumber("Pose X", getPose().getX());
    SmartDashboard.putNumber("Pose Y", getPose().getY());
    field.setRobotPose(getPose());
    fieldWheel.setRobotPose(new Pose2d(odometry2.getPoseMeters().getX(), -odometry2.getPoseMeters().getY(), getYaw()));
    
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
