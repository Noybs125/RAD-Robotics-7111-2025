// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.spark.SparkMax;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkLowLevel.MotorType;



import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {

  private int divID = 21;
  private SparkMax neoTestMotor = new SparkMax(divID, MotorType.kBrushless);
  private Command m_autonomousCommand;
  
  private int krakenID = 0;
  private TalonFX krakenTestMotor = new TalonFX(krakenID);

  private XboxController controller = new XboxController(0);

  private final RobotContainer m_robotContainer;

  public Robot() {
    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    
    double neoVoltage = neoTestMotor.getBusVoltage();

    if(controller.getAButton()){
      neoTestMotor.set(.5);
    } else if(controller.getYButton()){
      neoTestMotor.set(-0.5);
    }  else if(controller.getXButton()){
      neoTestMotor.set(-0.25);
    } else if(controller.getBButton()){
      neoTestMotor.set(-0.75);
    }
    else{
      neoTestMotor.set(0);
    }

    if(controller.getRightBumper()){
      krakenTestMotor.set(.5);
    } else if(controller.getLeftBumper()){
      krakenTestMotor.set(-.5);
    }else{
      krakenTestMotor.set(0);
    }

    SmartDashboard.putNumber("Can Bus Voltage", neoVoltage);
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}
}
