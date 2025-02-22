package frc.robot.utils.encoder;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

public class CANEncoder implements Encoder {
    private final CANcoder encoder;
    private double gearRatio = 1;
    private SensorDirectionValue isCW = SensorDirectionValue.Clockwise_Positive;

    /**
     * Constructor for the CANEncoder class. Also creates the encoder object.
     * Creats object using CANcoder class.
     * @param id -Id given to the encoder object.
     * @see -Link to CANcoder class (note not CANEncoder class): https://api.ctr-electronics.com/phoenix6/release/java/com/ctre/phoenix6/hardware/CANcoder.html.
     */
    public CANEncoder(int id) {
        encoder = new CANcoder(id);
    }

    /**
     * Constructor for the CANEncoder class.
     * Also creates encoder object, giving it a gear ratio calculation and a direction value along with its id.
     * Initilizes a Shuffleboard readout, Reading the voltage from the cancoder.
     * Creats object using CANcoder class.
     * @param id -Id givin to the encoder object.
     * @param gearRatio -Type "double", assigned to the encoder object as a property for calculations with position.
     * @param isCW -Type "SensorDirectionValue", isClockWise is a property assigned to the encoder object that serves as a direction for the sensor.
     * @see -Link to CANcoder class (note not CANEncoder class): https://api.ctr-electronics.com/phoenix6/release/java/com/ctre/phoenix6/hardware/CANcoder.html.
     */
    public CANEncoder(int id, double gearRatio, SensorDirectionValue isCW) {
        this.gearRatio = gearRatio;
        encoder = new CANcoder(id);
        this.isCW = isCW;

        Shuffleboard.getTab("DeviceOutputs").add("CANEncoderVolt", getVoltage()).withWidget("").getEntry();
    }

    /**
     * Gets the position of the encoder using the absolute position multiplied by its gear ratio.
     * Uses getAbsolutePosition and fromRotations(double) methods.
     * @return -Type "Rotation2d", Rotation object with the angle value as a property.
     * @see -Link to getAbsolutePosition: https://api.ctr-electronics.com/phoenix6/release/java/com/ctre/phoenix6/hardware/core/CoreCANcoder.html#getAbsolutePosition().
     * @see -Link to fromRotations(double): https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/math/geometry/Rotation2d.html#fromRotations(double).
     */
    public Rotation2d getPosition(){
        return Rotation2d.fromRotations(encoder.getAbsolutePosition().getValueAsDouble() * gearRatio);
    }

    /**
     * Gets the position of the encoder using the absolute position multiplied by its gear ratio. Default value is 0 rotations.
     * Uses getAbsolutePosition method.
     * @return -The angle of the encoder in rotations. Values range between -1 * gearRatio and 0.999755859375 * gearRatio.
     * @see -Link to getAbsolutePosition: https://api.ctr-electronics.com/phoenix6/release/java/com/ctre/phoenix6/hardware/core/CoreCANcoder.html#getAbsolutePosition().
     */
    public double getPositionAsDouble(){
         return encoder.getAbsolutePosition().getValueAsDouble() * gearRatio;
    }

    /**
     * Sets the encoder position to input, overwriting the current value to reset the position.
     * Uses setPosition method.
     * @param position -Type "Rotation2d", used to set the encoder equal to.
     * @see -Link to setPosition method: https://api.ctr-electronics.com/phoenix6/release/java/com/ctre/phoenix6/hardware/core/CoreCANcoder.html#setPosition(double).
     */
    public void setPosition(Rotation2d postion){
        encoder.setPosition(postion.getRotations());
    }

    /**
     * Gets if the encoder is an absolute value.
     * @return -True.
     * @deprecated
     */
    public boolean isAbsolute(){
        return true;
    }

    /**
     * Gets the rate of change for the encoder. Values range between -512 and 511.998046875. Default is 0.
     * Uses getVelocity method.
     * @return -Type "double", contains velocity in rotations per second.
     * @see -Link to getVelocity method: https://api.ctr-electronics.com/phoenix6/release/java/com/ctre/phoenix6/hardware/core/CoreCANcoder.html#getVelocity()
     */
    public double getVelocity(){
        return encoder.getVelocity().getValueAsDouble();
    }

    /**
     * Gets the supply voltage for the encoder
     * Uses getSupplyVoltage method.
     * @return -Type "double", contains voltage of encoder as a double between the range of 4 and 16. Default value is 4. 
     * @see -Link to getSupplyVoltage method: https://api.ctr-electronics.com/phoenix6/release/java/com/ctre/phoenix6/hardware/core/CoreCANcoder.html#getSupplyVoltage().
     */
    public double getVoltage(){
        return encoder.getSupplyVoltage().getValueAsDouble();
    }

    /**
     * Gets the current gear ratio for the encoder. This method allows other classes to read the current gear ratio.
     * @return Type "double", the current gear ratio used for calculating the position of the encoder.
     */
    public double getGearRatio() {
        return gearRatio;
    }

    /**
     * Sets the gear ratio for the encoder, overwriting the old ratio.
     */
    public void setGearRatio(double gearRatio){
        this.gearRatio = gearRatio;
    }

    /**
     * Zeros the encoder, setting the current position to 0.
     * Uses setPosition method.
     * @see -Link to setPosition method: https://api.ctr-electronics.com/phoenix6/release/java/com/ctre/phoenix6/hardware/core/CoreCANcoder.html#setPosition(edu.wpi.first.units.measure.Angle).
     */
    public void resetEncoder(){
        encoder.setPosition(0);
        return;
    }

    /**
     * OverWrites the is clockwise variable (isCW).
     * @param isCW -Type "boolean", sets which direcion increases the encoder, being true is clockwise positive, and false is counterclockwise positive.
     */
    public void setDirection(boolean isCW){
        if (isCW)
            this.isCW = SensorDirectionValue.Clockwise_Positive;
        else   
            this.isCW = SensorDirectionValue.CounterClockwise_Positive;
    }

    /**
     * Returns the current direction the encoder
     * @return -Type "boolean", true is clockwise positive, and false is counterclockwise positive.
     */
    public boolean getDirection(){
        if (isCW == SensorDirectionValue.Clockwise_Positive)
            return true;
        else 
            return false;
    }

    /**
     * Periodic method called 50 times per second. Currently completely empty.
     */
    public void periodic(){}
}
