package frc.robot.subsystems.climb;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.subsystems.climb.ClimbConstants.ClimbState;

public interface ClimbIO {

    @AutoLog
    class ClimbInputs{
        ClimbState state;
    }
    
    /**
     * Closes the climb
     */
    void close();

    /**
     * Opens the climb
     */
    void open();

    /**
     * 
     * @return Whether the climb is at setPoint
     */
    boolean isAtSetPoint();

    /**
     * Stops the motor
     */
    void stop();
    
    void update(ClimbInputs inputs);
    
}
