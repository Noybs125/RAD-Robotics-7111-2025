package frc.robot.utils.gyro;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.simulation.AnalogGyroSim;

public class SimGyro implements Gyro {

    private AnalogGyroSim gyroSim = new AnalogGyroSim(0);

    public SimGyro(){
        gyroSim.setInitialized(true);
    }

    @Override
    public Rotation2d get(RotationAxis axis) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'get'");
    }

    @Override
    public Rotation3d getRotation3d() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getRotation3d'");
    }

    @Override
    public void set(Rotation2d value, RotationAxis axis) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'set'");
    }

    @Override
    public void set(Rotation3d value) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'set'");
    }

    @Override
    public void setInverted(boolean isInverted) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setInverted'");
    }

    public void update(){

    }
    
}
