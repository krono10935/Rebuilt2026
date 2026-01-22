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
<<<<<<< HEAD
        motorLeft.setPercentOutput(IndexerConstants.LEFT_MOTOR_SPINNING_PERCENT_OUTPUT);
        motorRight.setPercentOutput(IndexerConstants.RIGHT_MOTOR_SPINNING_PERCENT_OUTPUT);
=======
        motor.setPercentOutput(IndexerConstants.SPINNING_PERCENT_OUTPUT);
>>>>>>> origin/Indexer
        isSpinning = true;
    }

    @Override
    public void turnOff() {
<<<<<<< HEAD
        motorLeft.stop();
        motorRight.stop();
=======
        motor.stop();
>>>>>>> origin/Indexer
        isSpinning = false;
    }

    @Override
    public void update(IndexerInputs inputs) {
        inputs.isOn = isSpinning;
    }
}
