// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Vision.ObjectDetection;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.utils.VirtualSubSystem;

/** Add your docs here. */
public class ObjectDetection extends VirtualSubSystem {
    private static ObjectDetection instance = null;

    private final ObjectDetectionIO io;
    private final ObjectDetectionInputsAutoLogged inputs;

    public static ObjectDetection getInstance(){
        if (instance == null){
            instance = new ObjectDetection();
        }
        return instance;
    }
    
    private ObjectDetection(){
        io = new ObjectDetectionIOPhoton();
        inputs = new ObjectDetectionInputsAutoLogged();
    }

    /**
     * if the camera sees any balls
     * if the camera is disconnected, it will return true
     * @return if the robot has balls
     */
    public boolean hasBalls(){
        return inputs.hasBalls || !Constants.USE_OBJECT_DETECTION ;
    }

    public Command waitUntilNoBalls(){
        return new WaitUntilCommand(() -> !this.hasBalls());
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("ObjectDetection", inputs);
    }
}
