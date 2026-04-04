package frc.robot.subsystems.Indexer;

import io.github.captainsoccer.basicmotor.controllers.Controller.ControlMode;
import io.github.captainsoccer.basicmotor.rev.BasicSparkMAX;

public class IndexerIOReal implements IndexerIO {
    private final BasicSparkMAX motorLeft;
    private final BasicSparkMAX motorRight;

    public IndexerIOReal( ){
        this.motorLeft = new BasicSparkMAX(IndexerConstants.getLeftMotorConfig());
        this.motorRight = new BasicSparkMAX(IndexerConstants.getRightMotorConfig());
    }

    @Override
    public void spinForward() {
        motorLeft.setControl(IndexerConstants.SPINNING_TARGET_VELOCITY, ControlMode.PROFILED_VELOCITY);
        motorRight.setControl(IndexerConstants.SPINNING_TARGET_VELOCITY, ControlMode.PROFILED_VELOCITY);
    }

    @Override
    public void spinBackward() {
        motorLeft.setControl(-IndexerConstants.SPINNING_TARGET_VELOCITY, ControlMode.PROFILED_VELOCITY);
        motorRight.setControl(-IndexerConstants.SPINNING_TARGET_VELOCITY, ControlMode.PROFILED_VELOCITY);
    }

    @Override
    public void turnOff() {
        motorLeft.stop();
        motorRight.stop();
    }

    @Override
    public void update(IndexerInputs inputs) {
        if (motorLeft.getController().getControlMode() == ControlMode.STOP || 
        motorLeft.getController().getGoal().velocity < IndexerConstants.SPEED_DEADBAND){
            inputs.isStuck = false;
        }

        inputs.isStuck = Math.abs(motorLeft.getController().getSetpointAsDouble()) < IndexerConstants.SPEED_DEADBAND ||
            Math.abs(motorRight.getController().getSetpointAsDouble()) < IndexerConstants.SPEED_DEADBAND;
    }

}
