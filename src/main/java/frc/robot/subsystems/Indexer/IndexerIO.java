package frc.robot.subsystems.Indexer;

import org.littletonrobotics.junction.AutoLog;

public interface IndexerIO {

    /**
     * sets the spindexer motor percent to the constant
     */
    void turnOn();

    /**
     * stops the motor
     */
    void turnOff();

    @AutoLog
    class IndexerInputs{
        boolean isOn;
    }

    void update(IndexerInputs inputs);


}
