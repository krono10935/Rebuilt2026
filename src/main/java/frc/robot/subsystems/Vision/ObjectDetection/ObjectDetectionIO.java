package frc.robot.subsystems.Vision.ObjectDetection;

import org.littletonrobotics.junction.AutoLog;

public interface ObjectDetectionIO {
    
    @AutoLog
    public class ObjectDetectionInputs{
        boolean isConnected;
        boolean hasBalls;
        boolean shotLastBall;
    }

    public void updateInputs(ObjectDetectionInputsAutoLogged inputs);
}
