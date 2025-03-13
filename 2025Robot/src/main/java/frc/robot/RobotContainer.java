package frc.robot;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.commands.PathfindThenFollowPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Deepclimb;
import frc.robot.subsystems.Field;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Field.FieldSetpoint;
import frc.robot.subsystems.Mechanisms.MechanismsState;
import frc.robot.subsystems.SuperStructure.ActualState;
import frc.robot.subsystems.SuperStructure.ControlState;
import frc.robot.subsystems.Flywheels;
import frc.robot.subsystems.Mechanisms;
import frc.robot.subsystems.Sensors;
import frc.robot.subsystems.SuperStructure;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

//import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  public final CommandXboxController operatorController;
  public final CommandXboxController driverController;

  public final Swerve swerve;
  public final Field field;
  public final Vision vision;
  public final Mechanisms mechanisms;
  public final Flywheels flywheels;
  public final SuperStructure superStructure;
  public final Sensors sensors;
  public final Deepclimb deepClimb;

  public final AHRS gyro = new AHRS(NavXComType.kMXP_SPI);

  private final SendableChooser<Command> autoChooser;

  public RobotContainer() {
    operatorController = new CommandXboxController(1);
    driverController = new CommandXboxController(0);
    

    sensors = new Sensors();
    vision = new Vision(gyro);
    swerve = new Swerve(gyro, vision);
    field = new Field(swerve);
    mechanisms = new Mechanisms();
    flywheels = new Flywheels();
    deepClimb = new Deepclimb();
    superStructure = new SuperStructure(swerve, vision, field, sensors, mechanisms, flywheels, deepClimb);

    
    NamedCommands.registerCommand("Coral Feeder", superStructure.setActualStateCommand(SuperStructure.ActualState.coralFeeder));
    NamedCommands.registerCommand("L1 Center", field.alignToNearestSetpoint(false, false).alongWith(superStructure.setActualStateCommand(SuperStructure.ActualState.coralL1Stow)));
    NamedCommands.registerCommand("L2 Left", field.alignToNearestSetpoint(true, false).andThen(superStructure.setActualStateCommand(SuperStructure.ActualState.coralL2)));
    NamedCommands.registerCommand("L2 Right", field.alignToNearestSetpoint(false, true).andThen(superStructure.setActualStateCommand(SuperStructure.ActualState.coralL2)));
    NamedCommands.registerCommand("L3 Left", field.alignToNearestSetpoint(true, false).andThen(superStructure.setActualStateCommand(SuperStructure.ActualState.coralL3)));
    NamedCommands.registerCommand("L3 Right", field.alignToNearestSetpoint(false, true).andThen(superStructure.setActualStateCommand(SuperStructure.ActualState.coralL3)));
    NamedCommands.registerCommand("L4 Left", field.alignToNearestSetpoint(true, false).andThen(superStructure.setActualStateCommand(SuperStructure.ActualState.coralL4)));
    NamedCommands.registerCommand("L4 Right", field.alignToNearestSetpoint(false, true).andThen(superStructure.setActualStateCommand(SuperStructure.ActualState.coralL4)));

    NamedCommands.registerCommand("Score", new InstantCommand(() -> flywheels.setSpeed(-0.6)));
    NamedCommands.registerCommand("Intake", new InstantCommand(() -> flywheels.setSpeed(1)));
    NamedCommands.registerCommand("Stop Wheels", new InstantCommand(() -> flywheels.setSpeed(0)));


    NamedCommands.registerCommand("L2 Algae", superStructure.useCenterAlignment().andThen(superStructure.setActualStateCommand(SuperStructure.ActualState.algaeL2)));
    NamedCommands.registerCommand("L3 Algae", superStructure.useCenterAlignment().andThen(superStructure.setActualStateCommand(SuperStructure.ActualState.algaeL3)));
    NamedCommands.registerCommand("Processor Algae", superStructure.setActualStateCommand(SuperStructure.ActualState.algaeProcessor));
    NamedCommands.registerCommand("Net Algae", superStructure.setActualStateCommand(SuperStructure.ActualState.algaeNet));

    autoChooser = AutoBuilder.buildAutoChooser();
    autoChooser.addOption("Custom", new PathPlannerAuto(field.generateAutoRoutine(field.getAutoCycles())));
    SmartDashboard.putData(autoChooser);

    // Configure button bindings
    configureButtonBindings();
  }
  
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // all of these bindings will be correctly defined when we decide controls
    Trigger leftReefAlign = driverController.leftTrigger().and(driverController.rightTrigger().negate());
    Trigger rightReefAlign = driverController.rightTrigger().and(leftReefAlign.negate());
    Trigger centerReefAlign = leftReefAlign.and(rightReefAlign);
    Trigger elevatorUp = operatorController.povUp();
    Trigger elevatorDown = operatorController.povDown();
    Trigger armUp = operatorController.povRight();
    Trigger armDown = operatorController.povLeft();
    Trigger effectorIntake = operatorController.rightStick();
    Trigger effectorScore = operatorController.leftStick();
    Trigger climbUp = driverController.povUp();
    Trigger climbDown = driverController.povDown();

    Trigger zeroGyro = driverController.start();

    Trigger l1 = operatorController.x();
    Trigger l2 = operatorController.a();
    Trigger l3 = operatorController.b();
    Trigger l4 = operatorController.y();
    Trigger stow = operatorController.back();

    Command leftAlign = field.alignToNearestSetpoint(true, false);
    Command rightAlign = field.alignToNearestSetpoint(false, true);
    Command centerAlign = field.alignToNearestSetpoint(false, false);

    swerve.setDefaultCommand(swerve.drive(
      () -> -swerve.getTransY(),
      () -> swerve.getTransX(),
      () -> swerve.getRotationZ(),
      () -> swerve.getFieldRelative(), 
      false
      )
    );

    elevatorUp.onTrue(new InstantCommand(() -> mechanisms.setElevatorSpeed(0.1))
        .alongWith(new InstantCommand(() -> superStructure.setActualState(ActualState.Manual))));

    elevatorDown.onTrue(new InstantCommand(() -> mechanisms.setElevatorSpeed(-0.1))
        .alongWith(new InstantCommand(() -> superStructure.setActualState(ActualState.Manual))));

    elevatorDown.negate().and(elevatorUp.negate()).onTrue(new InstantCommand(() -> mechanisms.setElevatorSpeed(0)));

    armUp.onTrue(new InstantCommand(() -> mechanisms.setWristSpeed(0.15))
        .alongWith(new InstantCommand(() -> superStructure.setActualState(ActualState.Manual))));

    armDown.onTrue(new InstantCommand(() -> mechanisms.setWristSpeed(-0.15))
        .alongWith(new InstantCommand(() -> superStructure.setActualState(ActualState.Manual))));

    armUp.negate().and(armDown.negate()).onTrue(new InstantCommand(() -> mechanisms.setWristSpeed(0)));

    effectorIntake.onTrue(new InstantCommand(() -> flywheels.setSpeed(1)));
    effectorScore.onTrue(new InstantCommand(() -> flywheels.setSpeed(-0.6)));
    effectorIntake.negate().and(effectorScore.negate()).onTrue(new InstantCommand(() -> flywheels.setSpeed(0)));

    climbUp.onTrue(new InstantCommand(() -> deepClimb.setSpeed(1)));
    climbDown.onTrue(new InstantCommand(() -> deepClimb.setSpeed(-1)));
    climbDown.negate().and(climbUp.negate()).onTrue(new InstantCommand(() -> deepClimb.setSpeed(0)));
    

    l1.onTrue(superStructure.setRobotStateCommand(ControlState.XButton));
    l2.onTrue(superStructure.setRobotStateCommand(ControlState.AButton));
    l3.onTrue(superStructure.setRobotStateCommand(ControlState.BButton));
    l4.onTrue(superStructure.setRobotStateCommand(ControlState.YButton));
    stow.onTrue(superStructure.setRobotStateCommand(ControlState.SelectButton));

    //leftReefAlign.onTrue(superStructure.useLeftAlignment());
    //rightReefAlign.onTrue(superStructure.useRightAlignment());
    //centerReefAlign.onTrue(superStructure.useCenterAlignment());
    //leftReefAlign.negate().and(rightReefAlign.negate()).and(centerReefAlign.negate()).onTrue(superStructure.useNoAlignment());
    rightReefAlign.onTrue(rightAlign).onFalse(new InstantCommand(() -> rightAlign.cancel()));
    leftReefAlign.onTrue(leftAlign).onFalse(new InstantCommand(() -> leftAlign.cancel()));
    centerReefAlign.onTrue(centerAlign).onFalse(new InstantCommand(() -> centerAlign.cancel()));

    // change or remove each of these when we decide controls
    zeroGyro.onTrue(swerve.zeroGyroCommand());
  }
}
