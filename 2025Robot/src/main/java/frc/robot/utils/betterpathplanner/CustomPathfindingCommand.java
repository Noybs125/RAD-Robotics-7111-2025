package frc.robot.utils.betterpathplanner;

import java.util.function.BiConsumer;
import java.util.function.Supplier;

import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PathFollowingController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.DriveFeedforwards;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class CustomPathfindingCommand extends PathfindingCommand {
    public CustomPathfindingCommand(
        Pose2d targetPose,
        PathConstraints constraints,
        double goalEndVel,
        Supplier<Pose2d> poseSupplier,
        Supplier<ChassisSpeeds> speedsSupplier,
        BiConsumer<ChassisSpeeds, DriveFeedforwards> output,
        PathFollowingController controller,
        RobotConfig robotConfig,
        Subsystem... requirements
    ){
        super(targetPose, constraints, goalEndVel, poseSupplier, speedsSupplier, output, controller, robotConfig, requirements);
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
