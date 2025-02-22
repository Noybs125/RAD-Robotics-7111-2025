package frc.robot.utils.encoder;

import edu.wpi.first.math.geometry.Rotation2d;

public interface Encoder {
    /**
     * Interface for getPosition for the encoders.
     * @return -The position of the encoders as rotation2d object with the angle in rotations being a property.
     */
    public Rotation2d getPosition();

    /**
     * Interface for getPositionAsDouble for the encoders.
     * @return -The position of the encoder as a double. The unit is rotations, 1 being 360 digrees.
     */
    public double getPositionAsDouble();

    /**
     * Interface for SetPosition for the encoders. Overwrites the current position value and replaces it with the input.
     * @param position -Type "Rotation2d", Unit is an object with rotations as a parameter.
     */
    public void setPosition(Rotation2d position);

    /**
     * Gets if the value of the encoders is absolute or relative.
     * @return -Type "boolean", true if the encoder is absolute, false if the encoder is relative.
     */
    public boolean isAbsolute();

    /**
     * Sets the gear ratio for the encoders, used for calculating the position of the encoder.
     * @param value
     */
    public void setGearRatio(double value);

    /**
     * @return -Type "double", gets the gear ratio for the encoders.
     */
    public double getGearRatio();

    /** 
     * Sets the direction for the encoders to increase.
     * @param isCW -Type "boolean", is clockwise (isCW). If true, clockwise positive. if false, counterclockwise positive.
     */
    public void setDirection(boolean isCW);

    /**
     * @return -Type "boolean", gets the direction that the encoder is set. If clockwise positive, returns true. If counterclockwise positive, returns false.
     */
    public boolean getDirection();

    /** must be called in the periodic method of the subsystem */
    public void periodic();
}
