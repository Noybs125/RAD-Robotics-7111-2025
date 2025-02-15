package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Auto extends SubsystemBase {

    private SendableChooser<Integer> alliance = new SendableChooser<Integer>();

    private Pose2d poseSetpoint = new Pose2d();

    Pose2d FeederLeft = new Pose2d(1.20, 70.3, Rotation2d.fromDegrees(306.59));
    Pose2d FeederRight = new Pose2d(1.20, 1.03, Rotation2d.fromDegrees(50.60)); 
    Pose2d CoralZone1 = new Pose2d(3.27, 4.03, Rotation2d.fromDegrees(0.00)); 
    Pose2d CoralZone2 = new Pose2d(3.87, 5.08, Rotation2d.fromDegrees(300.69)); 
    Pose2d CoralZone3 = new Pose2d(5.12, 5.11, Rotation2d.fromDegrees(237.01)); 
    Pose2d CoralZone4 = new Pose2d(5.72, 4.03, Rotation2d.fromDegrees(180)); 
    Pose2d CoralZone5 = new Pose2d(5.18, 2.78, Rotation2d.fromDegrees(119.01)); 
    Pose2d CoralZone6 = new Pose2d(3.76, 2.80, Rotation2d.fromDegrees(60.01)); 
    Pose2d Processor = new Pose2d(6.21, 0.39, Rotation2d.fromDegrees(270)); 
    Pose2d Barge = new Pose2d(7.90, 6.10, Rotation2d.fromDegrees(0.00));     
    public List<Pose2d> zoneMap = new ArrayList<>();

    public enum FieldSetpoints {
        Reef1,
        Reef2,
        Reef3,
        Reef4,
        Reef5,
        Reef6,
        Processor,
        SourceLeft,
        SourceRight,
        Barge,
        Climb
    }

    public Auto() {
        alliance.addOption("1", 1);
        alliance.addOption("2", 2);
        alliance.addOption("3", 3);
        zoneMap.add(FeederLeft);
        zoneMap.add(FeederRight);
        zoneMap.add(CoralZone1);
        zoneMap.add(CoralZone2);
        zoneMap.add(CoralZone3);
        zoneMap.add(CoralZone4);
        zoneMap.add(CoralZone5);
        zoneMap.add(CoralZone6);
        zoneMap.add(Processor);
        zoneMap.add(Barge);
    }

    public Command pathfindToPose(Pose2d pose) {
        if (DriverStation.getAlliance().isPresent()) {
            return DriverStation.getAlliance().get() == Alliance.Blue 
                ? AutoBuilder.pathfindToPose(pose, Constants.kAuto.constraints, 0)
                : AutoBuilder.pathfindToPoseFlipped(pose, Constants.kAuto.constraints, 0);
        }
        else return AutoBuilder.pathfindToPose(pose, Constants.kAuto.constraints, 0);
    }

    public Command pathfindToSetpoint(FieldSetpoints state) {
        switch (state) {
            case Reef1:
                poseSetpoint = new Pose2d(3.32, 4.02, Rotation2d.fromDegrees(0));
                break;
            case Reef2:
                poseSetpoint = new Pose2d(3.85, 5.11, Rotation2d.fromDegrees(-60));
                break;
            case Reef3:
                poseSetpoint = new Pose2d(5.09, 5.11, Rotation2d.fromDegrees(-120));
                break;
            case Reef4:
                poseSetpoint = new Pose2d(5.71, 4.02, Rotation2d.fromDegrees(-180));
                break;
            case Reef5:
                poseSetpoint = new Pose2d(5.11, 2.99, Rotation2d.fromDegrees(120));
                break;
            case Reef6:
                poseSetpoint = new Pose2d(3.88, 2.99, Rotation2d.fromDegrees(60));
                break;
            case SourceLeft:
                poseSetpoint = new Pose2d(1.07, 7.15, Rotation2d.fromDegrees(126));
                break;
            case SourceRight:
                poseSetpoint = new Pose2d(1.07, 0.96, Rotation2d.fromDegrees(-125.5));
                break;
            case Barge:
                poseSetpoint = new Pose2d(7.29, 6.18, Rotation2d.fromDegrees(0));
                break;
            case Climb:
                switch ((int) alliance.getSelected()) {
                    case 1:
                        poseSetpoint = new Pose2d(7.33, 7.27, Rotation2d.fromDegrees(0));
                        break;
                    case 2:
                        poseSetpoint = new Pose2d(7.33, 6.20, Rotation2d.fromDegrees(0));
                        break;
                    case 3:
                        poseSetpoint = new Pose2d(7.33, 5.09, Rotation2d.fromDegrees(0));
                        break;
                    default:
                        poseSetpoint = new Pose2d(4.49, 6.20, Rotation2d.fromDegrees(0));
                }
                break;
            case Processor:
                poseSetpoint = new Pose2d(6.13, 0.35, Rotation2d.fromDegrees(-90));
                break;

            default:
                poseSetpoint = new Pose2d(0, 0, Rotation2d.fromDegrees(0));
                break;
        }
    
        return pathfindToPose(poseSetpoint);

    }
    public Pose2d findZone(Pose2d robotPose){
        return robotPose.nearest(zoneMap);
    }

    public void periodic() {
    }
}
