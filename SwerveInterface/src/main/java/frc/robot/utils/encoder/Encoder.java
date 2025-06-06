package frc.robot.utils.encoder;

import edu.wpi.first.math.geometry.Rotation2d;

public interface Encoder {
    
    public Rotation2d getPosition();

    public void setPosition(Rotation2d rotation);
    
}
