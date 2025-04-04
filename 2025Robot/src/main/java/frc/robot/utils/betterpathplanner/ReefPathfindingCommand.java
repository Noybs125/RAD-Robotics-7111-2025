package frc.robot.utils.betterpathplanner;
import java.util.List;
import java.util.function.Supplier;

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Field;

public class ReefPathfindingCommand extends Command {
    private Supplier<Pose2d> startPose;
    private int reefFace;
    private Field field;
    private boolean isLeft;
    private Command command;

    public ReefPathfindingCommand(Supplier<Pose2d> startPose, int reefFace, boolean isLeft, Field field){
        this.startPose = startPose;
        this.reefFace = reefFace;
        this.field = field;
        this.isLeft = isLeft;
    }

    @Override
    public void initialize(){
        GoalEndState endState = new GoalEndState(0, field.zoneArray[reefFace-1].getRotation());
        List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(startPose.get(), field.transformPose(field.zoneArray[reefFace-1], isLeft, !isLeft));
        PathPlannerPath path = new PathPlannerPath(waypoints, Constants.kAuto.reefConstraints, null, endState);
        command = CustomAutoBuilder.followPath(path);
        command.initialize();
    }

    @Override
    public void execute(){
        command.execute();
        if(command.isFinished()){
            command.cancel();
        }
    }

    @Override
    public boolean isFinished() {
        return command.isFinished();
    }

    @Override
    public void end(boolean condition){
        command.end(condition);
    }

    @Override
    public void cancel(){
        command.cancel();
    }
}
