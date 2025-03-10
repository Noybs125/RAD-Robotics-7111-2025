package frc.robot;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Field;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Mechanisms.MechanismsState;
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

  public final XboxController xbox;
  public final CommandXboxController driverController;

  public final Swerve swerve;
  public final Field field;
  public final Vision vision;
  public final Mechanisms mechanisms;
  public final Flywheels flywheels;
  public final SuperStructure superStructure;
  public final Sensors sensors;
  //public final Deepclimb deepClimb;

  public final AHRS gyro = new AHRS(NavXComType.kMXP_SPI);

  private final SendableChooser<Command> autoChooser;

  public RobotContainer() {
    xbox = new XboxController(0);
    driverController = new CommandXboxController(0);
    

    sensors = new Sensors();
    vision = new Vision(gyro);
    swerve = new Swerve(gyro, vision);
    field = new Field(swerve);
    mechanisms = new Mechanisms();
    flywheels = new Flywheels();
    //deepClimb = new Deepclimb();
    superStructure = new SuperStructure(swerve, vision, field, sensors, mechanisms, flywheels, null);

    
    NamedCommands.registerCommand("Coral Feeder", superStructure.setActualStateCommand(SuperStructure.ActualState.coralFeeder));
    NamedCommands.registerCommand("L1 Center", superStructure.setActualStateCommand(SuperStructure.ActualState.coralL1).alongWith(superStructure.useCenterAlignment()));
    NamedCommands.registerCommand("L2 Left", superStructure.setActualStateCommand(SuperStructure.ActualState.coralL2).alongWith(superStructure.useLeftAlignment()));
    NamedCommands.registerCommand("L2 Right", superStructure.setActualStateCommand(SuperStructure.ActualState.coralL2).alongWith(superStructure.useRightAlignment()));
    NamedCommands.registerCommand("L3 Left", superStructure.setActualStateCommand(SuperStructure.ActualState.coralL3).alongWith(superStructure.useLeftAlignment()));
    NamedCommands.registerCommand("L3 Right", superStructure.setActualStateCommand(SuperStructure.ActualState.coralL3).alongWith(superStructure.useRightAlignment()));
    NamedCommands.registerCommand("L4 Left", superStructure.setActualStateCommand(SuperStructure.ActualState.coralL4).alongWith(superStructure.useLeftAlignment()));
    NamedCommands.registerCommand("L4 Right", superStructure.setActualStateCommand(SuperStructure.ActualState.coralL4).alongWith(superStructure.useRightAlignment()));

    NamedCommands.registerCommand("L2 Algae", superStructure.setActualStateCommand(SuperStructure.ActualState.algaeL2).alongWith(superStructure.useCenterAlignment()));
    NamedCommands.registerCommand("L3 Algae", superStructure.setActualStateCommand(SuperStructure.ActualState.algaeL3).alongWith(superStructure.useCenterAlignment()));
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
    Trigger elevatorUp = driverController.povUp();
    Trigger elevatorDown = driverController.povDown();
    Trigger armUp = driverController.povRight();
    Trigger armDown = driverController.povLeft();
    Trigger effectorIntake = driverController.rightStick();
    Trigger effectorScore = driverController.leftStick();

    Trigger zeroGyro = driverController.start();
    Trigger resetOdometry = driverController.a();

    Trigger l1 = driverController.x();
    Trigger l2 = driverController.a();
    Trigger l3 = driverController.b();
    Trigger l4 = driverController.y();
    Trigger stow = driverController.back();

    swerve.setDefaultCommand(swerve.drive(
      () -> -swerve.getTransY(),
      () -> swerve.getTransX(),
      () -> swerve.getRotationZ(),
      () -> swerve.getFieldRelative(), 
      false
      )
    );

    elevatorUp.onTrue(new InstantCommand(() -> mechanisms.setElevatorSpeed(0.1))
        .alongWith(new InstantCommand(() -> mechanisms.setState(MechanismsState.Manual))));

    elevatorDown.onTrue(new InstantCommand(() -> mechanisms.setElevatorSpeed(-0.1))
        .alongWith(new InstantCommand(() -> mechanisms.setState(MechanismsState.Manual))));

    elevatorDown.negate().and(elevatorUp.negate()).onTrue(new InstantCommand(() -> mechanisms.setElevatorSpeed(0)));

    armUp.onTrue(new InstantCommand(() -> mechanisms.setWristSpeed(0.1))
        .alongWith(new InstantCommand(() -> mechanisms.setState(MechanismsState.Manual))));

    armDown.onTrue(new InstantCommand(() -> mechanisms.setWristSpeed(-0.1))
        .alongWith(new InstantCommand(() -> mechanisms.setState(MechanismsState.Manual))));

    armUp.negate().and(armDown.negate()).onTrue(new InstantCommand(() -> mechanisms.setWristSpeed(0)));

    effectorIntake.onTrue(new InstantCommand(() -> flywheels.setSpeed(1)));
    effectorScore.onTrue(new InstantCommand(() -> flywheels.setSpeed(-0.25)));
    effectorIntake.negate().and(effectorScore.negate()).onTrue(new InstantCommand(() -> flywheels.setSpeed(0)));
    

    l1.onTrue(superStructure.setRobotStateCommand(ControlState.XButton));
    l2.onTrue(superStructure.setRobotStateCommand(ControlState.AButton));
    l3.onTrue(superStructure.setRobotStateCommand(ControlState.BButton));
    l4.onTrue(superStructure.setRobotStateCommand(ControlState.YButton));
    stow.onTrue(superStructure.setRobotStateCommand(ControlState.SelectButton));

    leftReefAlign.onTrue(superStructure.useLeftAlignment());
    rightReefAlign.onTrue(superStructure.useRightAlignment());
    centerReefAlign.onTrue(superStructure.useCenterAlignment());
    leftReefAlign.negate().and(rightReefAlign.negate()).and(centerReefAlign.negate()).onTrue(superStructure.useNoAlignment());

    // change or remove each of these when we decide controls
    zeroGyro.onTrue(swerve.zeroGyroCommand());
    resetOdometry.onTrue(swerve.resetOdometryCommand());
  }
}
