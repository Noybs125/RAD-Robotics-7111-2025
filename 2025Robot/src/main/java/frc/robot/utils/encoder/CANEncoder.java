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
     * @param position -Type "Rotation2d", 
     */
    public void setPosition(Rotation2d postion){
        encoder.setPosition(postion.getRotations());
    }

    public boolean isAbsolute(){
        return true;
    }

    public double getVelocity(){
        return encoder.getVelocity().getValueAsDouble();
    }

    public double getVoltage(){
        return encoder.getSupplyVoltage().getValueAsDouble();
    }

    public double getGearRatio() {
        return gearRatio;
    }

    public void setGearRatio(double gearRatio){
        this.gearRatio = gearRatio;
    }

    public void resetEncoder(){
        encoder.setPosition(0);
        return;
    }

    public void setDirection(boolean isCW){
        if (isCW)
            this.isCW = SensorDirectionValue.Clockwise_Positive;
        else   
            this.isCW = SensorDirectionValue.CounterClockwise_Positive;
    }

    public boolean getDirection(){
        if (isCW == SensorDirectionValue.Clockwise_Positive)
            return true;
        else 
            return false;
    }

    public void periodic(){

    }
    
}
