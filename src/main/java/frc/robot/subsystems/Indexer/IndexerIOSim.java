package frc.robot.subsystems.Indexer;

import io.github.captainsoccer.basicmotor.sim.motor.BasicMotorSim;

public class IndexerIOSim implements IndexerIO {
    private final BasicMotorSim motorLeft;
    private final BasicMotorSim motorRight;
    private boolean isSpinning;

    public IndexerIOSim() {
        this.motorLeft = new BasicMotorSim(IndexerConstants.getLeftMotorConfig());
        this.motorRight = new BasicMotorSim(IndexerConstants.getRightMotorConfig());
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
