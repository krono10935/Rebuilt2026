package frc.robot.subsystems.Indexer;

import io.github.captainsoccer.basicmotor.rev.BasicSparkMAX;

public class IndexerIOReal implements IndexerIO {
    private final BasicSparkMAX motorLeft;
    private final BasicSparkMAX motorRight;
    private boolean isSpinning;

    public IndexerIOReal( ){
        this.motorLeft = new BasicSparkMAX(IndexerConstants.getLeftMotorConfig());
        this.motorRight = new BasicSparkMAX(IndexerConstants.getRightMotorConfig());
        isSpinning = false;
    }

    @Override
    public void turnOn() {
        motorLeft.setPercentOutput(IndexerConstants.LEFT_MOTOR_SPINNING_PERCENT_OUTPUT);
        motorRight.setPercentOutput(IndexerConstants.RIGHT_MOTOR_SPINNING_PERCENT_OUTPUT);
        isSpinning = true;
    }

    @Override
    public void turnOff() {
        motorLeft.stop();
        motorRight.stop();
        isSpinning = false;
    }

    @Override
    public void update(IndexerInputs inputs) {
        inputs.isOn = isSpinning;
    }
}
