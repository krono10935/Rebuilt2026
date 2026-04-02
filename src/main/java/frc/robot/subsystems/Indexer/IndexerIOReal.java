package frc.robot.subsystems.Indexer;

import org.littletonrobotics.junction.Logger;

import io.github.captainsoccer.basicmotor.controllers.Controller.ControlMode;
import io.github.captainsoccer.basicmotor.rev.BasicSparkMAX;

public class IndexerIOReal implements IndexerIO {
    private final BasicSparkMAX motorLeft;
    private final BasicSparkMAX motorRight;

    public IndexerIOReal( ){
        this.motorLeft = new BasicSparkMAX(IndexerConstants.getLeftMotorConfig());
        this.motorRight = new BasicSparkMAX(IndexerConstants.getRightMotorConfig());
        Logger.recordOutput("Indexer/Mode", IndexerMode.STOPPED);
    }

    @Override
    public void turnOn() {
        motorLeft.setControl(IndexerConstants.SPINNING_TARGET_VELOCITY, ControlMode.PROFILED_VELOCITY);
        motorRight.setControl(IndexerConstants.SPINNING_TARGET_VELOCITY, ControlMode.PROFILED_VELOCITY);
        Logger.recordOutput("Indexer/Mode", IndexerMode.FORWARD);

    }

    @Override
    public void reverse() {
        motorLeft.setControl(-IndexerConstants.SPINNING_TARGET_VELOCITY, ControlMode.PROFILED_VELOCITY);
        motorRight.setControl(-IndexerConstants.SPINNING_TARGET_VELOCITY, ControlMode.PROFILED_VELOCITY);
        Logger.recordOutput("Indexer/Mode", IndexerMode.REVERSE);
    }

    @Override
    public void turnOff() {
        motorLeft.stop();
        motorRight.stop();
        Logger.recordOutput("Indexer/Mode", IndexerMode.STOPPED);
    }

    @Override
    public void update(IndexerInputs inputs) {
        if (motorLeft.getController().getControlMode() == ControlMode.STOP || 
        motorLeft.getController().getGoal().velocity < IndexerConstants.SPEED_DEADBAND){
            return;
        }

        inputs.isStuck = Math.abs(motorLeft.getController().getSetpointAsDouble()) < IndexerConstants.SPEED_DEADBAND ||
            Math.abs(motorRight.getController().getSetpointAsDouble()) < IndexerConstants.SPEED_DEADBAND;
    }

}
