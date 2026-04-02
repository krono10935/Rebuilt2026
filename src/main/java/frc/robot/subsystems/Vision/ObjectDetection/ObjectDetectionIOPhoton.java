package frc.robot.subsystems.Vision.ObjectDetection;

import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;

import edu.wpi.first.wpilibj.Timer;

public class ObjectDetectionIOPhoton implements ObjectDetectionIO{

    private PhotonCamera camera;
    
    private final Timer lastBallTimer = new Timer();


    public ObjectDetectionIOPhoton(){
        camera = new PhotonCamera(ObjectDetectionContstants.CAMERA_NAME);
        camera.setPipelineIndex(0);
    }


    @Override
    public void updateInputs(ObjectDetectionInputsAutoLogged inputs){ 
        try{
            inputs.isConnected = camera.isConnected();

            if(!inputs.isConnected){
                inputs.hasBalls = true;
                return;
            }

            var results = camera.getAllUnreadResults();
            if(results.isEmpty()){
                return;
            }

            var result = results.get(results.size() -1);


            inputs.hasBalls = false;
            for (var target : result.targets){
                if (target.area > ObjectDetectionContstants.MIN_AREA){
                    inputs.hasBalls = true;
                    inputs.shotLastBall = false;
                    lastBallTimer.stop();
                    lastBallTimer.reset();
                    return;
                }
            }
        }

        finally{
            Logger.recordOutput("ObjectDetection/lastBallTimer", lastBallTimer.get());

            if (!inputs.hasBalls && !inputs.shotLastBall && !lastBallTimer.isRunning()){
                lastBallTimer.start();
            } else if (lastBallTimer.hasElapsed(ObjectDetectionContstants.LAST_BALL_TIMEOUT)){
                inputs.shotLastBall = true;
                lastBallTimer.stop();
                lastBallTimer.reset();
            }
        }
    }

    

}