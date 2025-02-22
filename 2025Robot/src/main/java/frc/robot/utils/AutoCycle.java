package frc.robot.utils;


/**
 * This class represents a cycle in a custom auto routine.⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
 * It handles when the routine executes, which feeder station to intake from, the reef face, level, and branch to score on
 */
public class AutoCycle {

    public final int executedPosition;
    public final int feederStation;
    public final int reefFace;
    public final int levelToScore;
    public final boolean isLeft;

    /**
     * Initializes the Auto cycle
     * @param executedPosition The position in the sequential command group the command executes
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
}
