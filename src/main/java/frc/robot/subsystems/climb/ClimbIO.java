// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climb;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.subsystems.climb.ClimbConstants.ClimbState;

/** Add your docs here. */
public interface ClimbIO {

    @AutoLog
    class ClimbInputs{
        ClimbState state;
    }
    
    /**
     * closes the climb
     */
    void close();

    /**
     * opens the climb
     */
    void open();

    /**
     * 
     * @return if the climb is at setPoint
     */
    boolean isAtSetPoint();

    

    void update(ClimbInputs inputs);

}
