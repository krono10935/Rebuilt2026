// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Vision.ObjectDetection;

import org.littletonrobotics.junction.AutoLog;
import org.photonvision.PhotonCamera;

import frc.utils.VirtualSubSystem;

/** Add your docs here. */
public class ObjectDetection extends VirtualSubSystem{

     @AutoLog
    public static class ObjectDetectionInputs{
        public boolean isConnected;
        public boolean hasBalls;
        public boolean isEnabled;
    }

    private final PhotonCamera camera;
    private final ObjectDetectionInputsAutoLogged inputs = new ObjectDetectionInputsAutoLogged();
    
    public ObjectDetection(){
        camera = new PhotonCamera(ObjectDetectionContstans.CAMERA_NAME);
    }

    /**
     * enables the object detection camera
     */
    public void enableCamera(){
        inputs.isEnabled = true;
        camera.setPipelineIndex(1);
    }

    /**
     * disables the object detection camera, to save on prossesing power
     */
    public void disableCamera(){
        inputs.isEnabled = false;
        camera.setPipelineIndex(0);
    }

    /**
     * if the camera sees any balls
     * if the camera is disconnected, it will return true
     * @return if the robot has balls
     */
    public boolean hasBalls(){
        return inputs.hasBalls;
    }

    @Override
    public void periodic() {
        inputs.isConnected = camera.isConnected();
        if(!inputs.isConnected){
            inputs.hasBalls = true;
            return;
        }

        var results = camera.getAllUnreadResults();
        if(!inputs.isEnabled){
            return;
        }
        

        if(results.isEmpty()){
            inputs.hasBalls = false;
            return;
        }

        var result = results.get(results.size() -1);
        inputs.hasBalls = result.targets.size() > 0;
    }
}
