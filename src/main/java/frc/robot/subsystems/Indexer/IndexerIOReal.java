package frc.robot.subsystems.Indexer;

import java.util.ResourceBundle.Control;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import io.github.captainsoccer.basicmotor.controllers.Controller.ControlMode;
import io.github.captainsoccer.basicmotor.rev.BasicSparkMAX;

public class IndexerIOReal implements IndexerIO {
    private final BasicSparkMAX motorLeft;
    private final BasicSparkMAX motorRight;
    private boolean isSpinning;

    public IndexerIOReal( ){
        this.motorLeft = new BasicSparkMAX(IndexerConstants.getLeftMotorConfig());
        this.motorRight = new BasicSparkMAX(IndexerConstants.getRightMotorConfig());

        SmartDashboard.putData(motorLeft.getController());
        SmartDashboard.putData(motorRight.getController());

        isSpinning = false;
    }

    @Override
    public void turnOn() {
        motorLeft.setControl(IndexerConstants.SPINNING_TARGET_VELOCITY, ControlMode.PROFILED_VELOCITY);
        motorRight.setControl(IndexerConstants.SPINNING_TARGET_VELOCITY, ControlMode.PROFILED_VELOCITY);
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
