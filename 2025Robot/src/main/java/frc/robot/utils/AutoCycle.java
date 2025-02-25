package frc.robot.utils;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

/**
 * This class represents a cycle in a custom auto routine.⠀
 * It handles when the routine executes, which feeder station to intake from, the reef face, level, and branch to score on
 */
public class AutoCycle {

    public int executedPosition;
    public int feederStation;
    public int reefFace;
    public int levelToScore;
    public boolean isLeft;

    private SendableChooser<Integer> coralStationChooser = new SendableChooser<>();
    private SendableChooser<Integer> reefFaceChooser = new SendableChooser<>();
    private SendableChooser<Boolean> reefBranchChooser = new SendableChooser<>();
    private SendableChooser<Integer> reefLevelChooser = new SendableChooser<>();

    private int[] coralStations = new int[] {};
    private int[] reefFaces = new int[] {};
    private int[] reefLevels = new int[] {};

    /**
     * Initializes the Auto cycle
     * @param executedPosition The position in the sequential command group the command executes
     * @param feederStation The feeder station to intake the gamepiece from, 1 = left, 2 = right, 0 = preload
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

        coralStationChooser.addOption("Preload", 0);
        coralStationChooser.addOption("Left", 1);
        coralStationChooser.addOption("Right", 2);

        for(var face : reefFaces){
            reefFaceChooser.addOption("" + face, face);
        }
        for(var level : reefLevels){
            reefFaceChooser.addOption("" + level, level);
        }

        reefBranchChooser.addOption("Left", true);
        reefBranchChooser.addOption("Right", false);
    }

    public void setAutoCycle(){
        feederStation = coralStationChooser.getSelected();
        reefFace = reefFaceChooser.getSelected();
        levelToScore = reefLevelChooser.getSelected();
        reefFace = reefFaceChooser.getSelected();
    }
}
