package frc.robot.subsystems.Indexer;

import io.github.captainsoccer.basicmotor.sim.motor.BasicMotorSim;

public class IndexerIOSim implements IndexerIO {
    private final BasicMotorSim motor;
    private boolean isSpinning;

    public IndexerIOSim() {
        this.motor = new BasicMotorSim(IndexerConstants.getConfig());
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
