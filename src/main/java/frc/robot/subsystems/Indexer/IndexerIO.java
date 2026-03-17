package frc.robot.subsystems.Indexer;

import org.littletonrobotics.junction.AutoLog;

public interface IndexerIO {
    
    @AutoLog
    class IndexerInputs{
        boolean isOn;
    }

    /**
     * sets the spindexer motor percent to the constant
     */
    void turnOn();

    /**
     * stops the motor
     */
    void turnOff();

    /*
     * @return if the indexer is stuck
     */
    boolean isStuck();

    void update(IndexerInputs inputs);
}
