package frc.robot.subsystems.Indexer;

import org.littletonrobotics.junction.AutoLog;

public interface IndexerIO {
    
    @AutoLog
    class IndexerInputs{
        boolean isStuck;
    }

    public enum IndexerMode{
        REVERSE,
        STOPPED,
        FORWARD
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

    void update(IndexerInputs inputs);
}
