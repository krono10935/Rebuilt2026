package frc.robot.subsystems.Indexer;

import io.github.captainsoccer.basicmotor.BasicMotor;
import org.littletonrobotics.junction.AutoLog;

public interface IndexerIO {

    void start();

    void stop();


    @AutoLog
    class IndexerInputs{
        boolean isOn;
    }

    void update(IndexerInputs inputs);


}
