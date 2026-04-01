// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Vision.ObjectDetection;

import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.utils.VirtualSubSystem;

/** Add your docs here. */
public class ObjectDetection extends VirtualSubSystem {
    private static ObjectDetection instance = null;

    private boolean isConnected;
    private boolean hasBalls;
    private boolean isEnabled;

    private boolean shotLastBall;
    private final Timer lastBallTimer = new Timer();


    private final PhotonCamera camera;

    public static ObjectDetection getInstance(){
        if (instance == null){
            instance = new ObjectDetection();
        }
        return instance;
    }
    
    private ObjectDetection(){
        camera = new PhotonCamera(ObjectDetectionContstants.CAMERA_NAME);
        enableCamera();
    }

    /**
     * enables the object detection camera
     */
    public void enableCamera(){
        isEnabled = true;
        camera.setPipelineIndex(0);
    }

    /**
     * disables the object detection camera, to save on prossesing power
     */
    public void disableCamera(){
        isEnabled = false;
        camera.setPipelineIndex(1);
    }

    /**
     * if the camera sees any balls
     * if the camera is disconnected, it will return true
     * @return if the robot has balls
     */
    public boolean hasBalls(){
        return hasBalls || !Constants.USE_OBJECT_DETECTION ;
    }

    public Command waitUntilNoBalls(){
        return new WaitUntilCommand(() -> !this.hasBalls());
    }

    @Override
    public void periodic() {
        try{
            isConnected = camera.isConnected();

            if(!isConnected){
                hasBalls = true;
                return;
            }

            var results = camera.getAllUnreadResults();
            if(!isEnabled || results.isEmpty()){
                return;
            }

            var result = results.get(results.size() -1);


            hasBalls = false;
            for (var target : result.targets){
                if (target.area > ObjectDetectionContstants.MIN_AREA){
                    hasBalls = true;
                    shotLastBall = false;
                    lastBallTimer.stop();
                    lastBallTimer.reset();
                    return;
                }
            }
        }

        finally{
            Logger.recordOutput("ObjectDetection/is connected", isConnected);
            Logger.recordOutput("ObjectDetection/has balls", hasBalls);
            Logger.recordOutput("ObjectDetection/is Enabled", isEnabled);
            Logger.recordOutput("ObjectDetection/timer", lastBallTimer.get());
            Logger.recordOutput("ObjectDetection/shotLastBall", shotLastBall);

            if (!hasBalls && !shotLastBall && !lastBallTimer.isRunning()){
                lastBallTimer.start();
            } else if (lastBallTimer.hasElapsed(ObjectDetectionContstants.LAST_BALL_TIMEOUT)){
                shotLastBall = true;
                lastBallTimer.stop();
                lastBallTimer.reset();
            }
        }
    }
}
