package frc.robot.utils;

import edu.wpi.first.wpilibj2.command.Command;

/**
 * This class handles a cycle in a custom auto routine.⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
 * It handles when the routine executes, which feeder station to intake from, the reef face, level, and branch to score on
 */
public class AutoCycle {

    private final int executedPosition;
    private final int feederStation;
    private final int reefFace;
    private final int levelToScore;
    private final boolean isLeft;

    /**
     * Initializes the Auto cycle
     * @param executedPosition The place in the sequential command group the command executes
     * @param feederStation The feeder station to intake the gamepiece from, 1 = left, 2 = right, 0 = start config
     * @param reefFace The side of the reef (1 - 6) to score the gamepiece
     * @param levelToScore The reef level (1-4) to score the gamepiece on
     * @param isLeft Whether you score on the left or right branch of the reef face
     */
    public AutoCycle(int executedPosition, int feederStation, int reefFace, int levelToScore, boolean isLeft){
        this.executedPosition = executedPosition;
        this.feederStation = feederStation;
        this.isLeft = isLeft;
        this.levelToScore = levelToScore;
        this.reefFace = reefFace;
    }

    public int getExecutedPosition(){
        return executedPosition;
    }

    public Command getCommand(){
        return null;
    }
}
