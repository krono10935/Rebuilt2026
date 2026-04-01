package frc.robot.subsystems.Indexer;

import org.littletonrobotics.junction.AutoLog;

public interface IndexerIO {
    
    @AutoLog
    class IndexerInputs{
        boolean isOn;
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

    /**
     * @return Whether the indexer is stuck
     */
    boolean isStuck();

    void setSpeed(double speedRps);

    void update(IndexerInputs inputs);
}
