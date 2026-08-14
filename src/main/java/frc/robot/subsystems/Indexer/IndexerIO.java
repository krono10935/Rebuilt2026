package frc.robot.subsystems.Indexer;

import org.littletonrobotics.junction.AutoLog;

public interface IndexerIO {
    
    @AutoLog
    class IndexerInputs{
        boolean isStuck;
    }


    /**
     * Sets the spindexer motor percent to the constant
     */
    void spinForward();

    /**
     * Sets the spindexer motor percent to the constant but reverse
     */
    void spinBackward();

    /**
     * Stops the motor
     */
    void turnOff();

    void update(IndexerInputs inputs);

    public class IndexerIOReplay implements IndexerIO{

        @Override
        public void spinForward() {

        }

        @Override
        public void spinBackward() {

        }

        @Override
        public void turnOff() {

        }

        @Override
        public void update(IndexerInputs inputs) {

        }
        
    }
}
