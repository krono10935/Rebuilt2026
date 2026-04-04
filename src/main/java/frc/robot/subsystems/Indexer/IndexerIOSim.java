package frc.robot.subsystems.Indexer;

import io.github.captainsoccer.basicmotor.controllers.Controller.ControlMode;
import io.github.captainsoccer.basicmotor.sim.motor.BasicMotorSim;

public class IndexerIOSim implements IndexerIO {
    private final BasicMotorSim motorLeft;
    private final BasicMotorSim motorRight;

    public IndexerIOSim( ){
        this.motorLeft = new BasicMotorSim(IndexerConstants.getLeftMotorConfig());
        this.motorRight = new BasicMotorSim(IndexerConstants.getRightMotorConfig());
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
            return;
        }

        inputs.isStuck = Math.abs(motorLeft.getController().getSetpointAsDouble()) < IndexerConstants.SPEED_DEADBAND ||
            Math.abs(motorRight.getController().getSetpointAsDouble()) < IndexerConstants.SPEED_DEADBAND;
    }

}