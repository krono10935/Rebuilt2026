package frc.robot.subsystems.Indexer;

import io.github.captainsoccer.basicmotor.controllers.Controller.ControlMode;
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
        motorLeft.setControl(IndexerConstants.SPINNING_TARGET_VELOCITY, ControlMode.PROFILED_VELOCITY);
        motorRight.setControl(IndexerConstants.SPINNING_TARGET_VELOCITY, ControlMode.PROFILED_VELOCITY);
        isSpinning = true;
    }

    @Override
    public void reverse() {
        motorLeft.setControl(-IndexerConstants.SPINNING_TARGET_VELOCITY, ControlMode.PROFILED_VELOCITY);
        motorRight.setControl(-IndexerConstants.SPINNING_TARGET_VELOCITY, ControlMode.PROFILED_VELOCITY);
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

    @Override
    public boolean isStuck() {
        return Math.abs(motorLeft.getController().getSetpointAsDouble()) < IndexerConstants.SPEED_DEADBAND ||
            Math.abs(motorRight.getController().getSetpointAsDouble()) < IndexerConstants.SPEED_DEADBAND;
    }

    @Override
    public void setSpeed(double speedRps){
        motorLeft.setControl(speedRps,ControlMode.VELOCITY);
        motorRight.setControl(speedRps,ControlMode.VELOCITY);
    }
}
