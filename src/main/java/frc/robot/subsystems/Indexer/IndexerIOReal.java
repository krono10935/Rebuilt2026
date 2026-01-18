package frc.robot.subsystems.Indexer;

import io.github.captainsoccer.basicmotor.rev.BasicSparkMAX;

public class IndexerIOReal implements IndexerIO {
    private final BasicSparkMAX motor;
    private boolean isSpinning;

    public IndexerIOReal( ){
        this.motor = new BasicSparkMAX(IndexerConstants.getConfig());
        isSpinning = false;
    }

    @Override
    public void start() {
        motor.setPercentOutput(IndexerConstants.SPINNING_PERCENT_OUTPUT);
        isSpinning = true;
    }

    @Override
    public void stop() {
        motor.stop();
        isSpinning = false;
    }

    @Override
    public void update(IndexerInputs inputs) {
        inputs.isOn = isSpinning;
    }
}
