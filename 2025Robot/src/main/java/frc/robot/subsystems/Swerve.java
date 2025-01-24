package frc.robot.subsystems;

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
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
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
  public RobotConfig config;

  private final AHRS gyro;
  private final Vision vision;
  private ComplexWidget fieldPublisher;
  

  public Swerve(AHRS gyro, Vision vision) {
    this.gyro = gyro;
    this.vision = vision;
    fieldPublisher = Shuffleboard.getTab("Odometry").add("field odometry", field).withWidget("Field");
    zeroGyro();
    

    modules = new SwerveModule[] {
      new SwerveModule(0, Constants.kSwerve.MOD_0_Constants),
      new SwerveModule(1, Constants.kSwerve.MOD_1_Constants),
      new SwerveModule(2, Constants.kSwerve.MOD_2_Constants),
      new SwerveModule(3, Constants.kSwerve.MOD_3_Constants),
    };
    odometry2 = new SwerveDriveOdometry(Constants.kSwerve.KINEMATICS, getYaw(), getPositions());
    swerveOdometry = new SwerveDrivePoseEstimator(Constants.kSwerve.KINEMATICS, getYaw(), getPositions(),vision.robotPose);

    try{
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      
      e.printStackTrace();
    }
    
    AutoBuilder.configure(
            this::getPose, // Robot pose supplier
            this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getRelSpeedsNonSuplier, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            (speeds, feeds) -> driveRobotRelative(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
            Constants.kAuto.cont,
            config,
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
  public Command drive(DoubleSupplier forwardBackAxis, DoubleSupplier leftRightAxis, DoubleSupplier rotationAxis, boolean isFieldRelative, boolean isOpenLoop) {
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

      // Get desired module states.
      ChassisSpeeds chassisSpeeds = isFieldRelative
        ? ChassisSpeeds.fromFieldRelativeSpeeds(forwardBack, leftRight, rotation, getYaw().unaryMinus())
        : new ChassisSpeeds(forwardBack, leftRight, rotation);

      SwerveModuleState[] states = Constants.kSwerve.KINEMATICS.toSwerveModuleStates(chassisSpeeds);

      setModuleStates(states, isOpenLoop);
    }).withName("SwerveDriveCommand");
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
    return gyro.getRotation2d(); //Rotation2d.fromDegrees(gyro.getYaw());
  }

  public Command zeroGyroCommand() {
    return runOnce(this::zeroGyro).withName("ZeroGyroCommand");
  }

  private void zeroGyro() {
    gyro.zeroYaw();
  }

  public Pose2d getPose() {
    return swerveOdometry.getEstimatedPosition();
  }

  public void resetOdometry(Pose2d pose) { // not currently used, using addVisionMeasurements in periodic instead.
      swerveOdometry.resetPose(pose);    
  }
  private static void printTrace(StackTraceElement[] trace)
    {
        for (StackTraceElement t : trace)
        {
            System.out.println(t);
        }
    }

  public ChassisSpeeds getRelSpeedsNonSuplier() {
    ChassisSpeeds relSpeed = Constants.kSwerve.KINEMATICS.toChassisSpeeds(getStates());
    return relSpeed;
  }

  public void driveRobotRelative(ChassisSpeeds speeds){
    StackTraceElement[] trace = Thread.currentThread().getStackTrace();
    System.out.println("Spacer!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
    printTrace(trace);

    speeds = ChassisSpeeds.discretize(speeds, 0.02);
    SwerveModuleState[] states = Constants.kSwerve.KINEMATICS.toSwerveModuleStates(speeds);
    setModuleStates(states);

  }
  
  @Override 
  public void periodic() {
      //swerveOdometry.update(getYaw(), getPositions());
    /*for(Camera camera : vision.cameraList){
      if(camera.updatePose()){
        swerveOdometry.addVisionMeasurement(camera.getRobotPose(), Timer.getFPGATimestamp(), camera.getPoseAmbiguity());
      }
    }*/
    
    for(SwerveModule mod : modules){
      SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Cancoder", mod.getCanCoderDegrees().getDegrees());
      SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Integrated", mod.getPosition().angle.getDegrees());
      SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);    
    }
    SmartDashboard.putNumber("Gyro Yaw", getYaw().getDegrees());
    field.setRobotPose(getPose());
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
