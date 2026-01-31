package frc.robot.subsystems.Indexer;

import io.github.captainsoccer.basicmotor.rev.BasicSparkMAX;

public class IndexerIOReal implements IndexerIO {
    private final BasicSparkMAX motorLeft;
    private final BasicSparkMAX motorRight;
    private boolean isSpinning;

    public IndexerIOReal( ){
        this.motorLeft = new BasicSparkMAX(IndexerConstants.getLeftMotorConfig());
        this.motorRight = new BasicSparkMAX(IndexerConstants.getRightMotorConfig());

        motorRight.followMotor(motorLeft, true);
        isSpinning = false;
    }

    @Override
    public void turnOn() {
        motorLeft.setPercentOutput(IndexerConstants.SPINNING_PRECENT_OUTPUT);
        isSpinning = true;
    }

    @Override
    public void turnOff() {
        motorLeft.stop();
        isSpinning = false;
    }

    @Override
    public void update(IndexerInputs inputs) {
        inputs.isOn = isSpinning;
    }
}
