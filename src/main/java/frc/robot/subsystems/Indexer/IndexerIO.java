package frc.robot.subsystems.Indexer;

import org.littletonrobotics.junction.AutoLog;

import io.github.captainsoccer.basicmotor.controllers.Controller.ControlMode;

public interface IndexerIO {
    
    @AutoLog
    class IndexerInputs{
        ControlMode controlMode;
        double targetSpeedMPS;
        double motorLeftSpeedMPS;
        double motorRightSpeedMPS;
    }

    /**
     * Sets the spindexer motor percent to the constant
     */
    void turnOn();

    /**
     * Sets the spindexer motor percent to the constant but reverse
     */
    void reverse();

    /**
     * Stops the motor
     */
    void turnOff();


    void setSpeed(double speedRps);

    void update(IndexerInputs inputs);
}
