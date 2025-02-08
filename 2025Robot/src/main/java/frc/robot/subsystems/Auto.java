package frc.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;

public class Auto {

    private Pose2d poseSetpoint;
    private FieldSetpoints state;

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

    public Command pathfindToPose(Pose2d pose) {
        return AutoBuilder.pathfindToPose(pose, Constants.kAuto.constraints, 0);
    }

    public Command pathfindToSetpoint(FieldSetpoints state) {
        switch (state) {
            case Reef1:
                break;
            case Reef2:
                break;
            case Reef3:
                break;
            case Reef4:
                break;
            case Reef5:
                break;
            case Reef6:
                break;
            case SourceLeft:
                break;
            case SourceRight:
                break;
            case Barge:
                break;
            case Climb:
                break;
            case Processor:
                poseSetpoint = new Pose2d(0, 0, Rotation2d.fromDegrees(0));
                break;

            default:
                poseSetpoint = new Pose2d(0, 0, Rotation2d.fromDegrees(0));
                break;
        }
    
        return pathfindToPose(poseSetpoint);

    }
}
