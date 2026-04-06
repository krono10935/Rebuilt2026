package frc.robot.subsystems.Vision.ObjectDetection;

public class ObjectDetectionIOSim implements ObjectDetectionIO{
    public ObjectDetectionIOSim(){

    }


    @Override
    public void updateInputs(ObjectDetectionInputsAutoLogged inputs){ 
        inputs.hasBalls = true;
        inputs.isConnected = true;
    }


}