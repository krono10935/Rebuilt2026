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
    public void turnOn() {
        motorLeft.setControl(IndexerConstants.SPINNING_TARGET_VELOCITY, ControlMode.PROFILED_VELOCITY);
        motorRight.setControl(IndexerConstants.SPINNING_TARGET_VELOCITY, ControlMode.PROFILED_VELOCITY);
    }

    @Override
    public void reverse() {
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
        inputs.targetSpeedMPS = motorLeft.getController().getGoalAsDouble();
        inputs.motorLeftSpeedMPS = motorLeft.getVelocity();
        inputs.motorRightSpeedMPS = motorRight.getVelocity();

        inputs.controlMode = motorLeft.getController().getControlMode();
    }


    @Override
    public void setSpeed(double speedRps){
        motorLeft.setControl(speedRps,ControlMode.VELOCITY);
        motorRight.setControl(speedRps,ControlMode.VELOCITY);
    }
}
