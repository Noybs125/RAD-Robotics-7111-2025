package frc.robot.subsystems;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import com.studica.frc.AHRS;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import frc.robot.Constants;
import frc.robot.utils.Camera;

public class Vision extends SubsystemBase{
    
    private final AHRS gyro;
    public Pose2d robotPose = new Pose2d();
    public Pose3d estPose3d = new Pose3d();
    private Transform2d tagAllignment = new Transform2d();
    private VisionState state = VisionState.Climb;

    // TODO: change variable names on actual robot
    public final Camera limelight = new Camera(
        new PhotonCamera("photonvision"), 
        Constants.vision.cameraToRobotCenter1, 
        new EstimatedRobotPose(estPose3d, 0.0, null, PoseStrategy.AVERAGE_BEST_TARGETS), 
        this
        );
    public final Camera orangepi1 = new Camera(
        new PhotonCamera("OV9281_1"), 
        Constants.vision.cameraToRobotCenter2, 
        new EstimatedRobotPose(estPose3d, 0.0, null, PoseStrategy.AVERAGE_BEST_TARGETS), 
        this
        );
    public final Camera orangepi2 = new Camera(
        new PhotonCamera("OV9281_3"), 
        Constants.vision.cameraToRobotCenter3, 
        new EstimatedRobotPose(estPose3d, 0.0, null, PoseStrategy.AVERAGE_BEST_TARGETS), 
        this
        );

    public Camera[] cameraList = new Camera[] {
        //orangepi1,
        //orangepi2,
    };

    public enum VisionState{
        LeftReef,
        RightReef,
        ReefL1,
        FeederStation,
        Processor,
        Climb,
    }

    public Vision(AHRS gyro){
        this.gyro = gyro;
    }

    public void setState(VisionState state){
        this.state = state;
    }

    private void handleState(){
        switch (state) {
            case LeftReef:
                tagAllignment = new Transform2d(0, 0, Rotation2d.fromDegrees(0));
                break;

            case RightReef:
                tagAllignment = new Transform2d(0, 0, Rotation2d.fromDegrees(0));
                break;

            case ReefL1:
                tagAllignment = new Transform2d(0, 0, Rotation2d.fromDegrees(0));
                break;

            case FeederStation:
                tagAllignment = new Transform2d(0, 0, Rotation2d.fromDegrees(0));
                break;

            case Processor:
                tagAllignment = new Transform2d(0, 0, Rotation2d.fromDegrees(0));
                break;

            case Climb:
                tagAllignment = new Transform2d(0, 0, Rotation2d.fromDegrees(0));
                break;
                
            default:
                break;
        }
    }

    public void periodic(){

        Optional<EstimatedRobotPose> estPose = null;

         for(Camera camera : cameraList){
            if(camera.getEstimatedGlobalPose(robotPose) != null){
                estPose = camera.getEstimatedGlobalPose(robotPose);
            }
            robotPose = camera.estRobotPose.estimatedPose.transformBy(camera.getCameraToRobot()).toPose2d();
            if(estPose != null){
                if(estPose.isPresent()){
                    camera.estRobotPose = estPose.get();
                }
            }

            camera.periodic();
        }
        handleState();
    }
}
