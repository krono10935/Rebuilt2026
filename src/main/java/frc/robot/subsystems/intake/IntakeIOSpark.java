package frc.robot.subsystems.intake;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import io.github.captainsoccer.basicmotor.BasicMotor;
import io.github.captainsoccer.basicmotor.controllers.Controller.ControlMode;
import io.github.captainsoccer.basicmotor.rev.BasicSparkMAX;

public class IntakeIOSpark implements IntakeIO {
    private final BasicMotor intakeMotor;
    private final BasicMotor positionMotor;

    public IntakeIOSpark() {

        intakeMotor = new BasicSparkMAX(IntakeConstants.intakeMotorConfig);

        positionMotor = new BasicSparkMAX(IntakeConstants.positionMotorConfig);

    }

    @Override
    public boolean intakeMotorAtSetPoint() {
        return intakeMotor.atSetpoint();
    }

    @Override
    public void setIntakeMotorVelocity(Rotation2d velocity) {
        intakeMotor.setControl(velocity.getRotations(), ControlMode.VELOCITY);
    }

    @Override
    public void stopIntakeMotor() {
        intakeMotor.stop();
    }

    @Override
    public void stopIntakeOpeningMotor() {
        positionMotor.stop();
    }

    @Override
    public void setPositionMotorPercentOutput(double percent){
        positionMotor.setPercentOutput(percent);
    }

    @Override
    public boolean positionMotorAtSetPoint() {
        return positionMotor.atSetpoint();
    }

    @Override
    public void setPositionMotorVelocity(Rotation2d velocity){
        positionMotor.setControl(velocity.getRotations(), ControlMode.VELOCITY);
    }

    @Override
    public double getIntakePosition() {
        return positionMotor.getPosition();
    }

    @Override
    public void setPositionMotor(double pos) {
        positionMotor.setControl(pos, ControlMode.POSITION);
    }

    @Override
    public Rotation2d getSpeedPositionMotor() {
        return Rotation2d.fromRotations(positionMotor.getVelocity());
    }


    @Override
    public void updateInputs(IntakeInputs inputs) {
        inputs.position = positionMotor.getPosition();
        inputs.power = IntakeConstants.INTAKE_KT 
            * intakeMotor.getSensorData().currentOutput()
            * Units.rotationsPerMinuteToRadiansPerSecond(intakeMotor.getVelocity());
        inputs.velocity = intakeMotor.getVelocity(); 
    }

    @Override
    public boolean isInPositionControl() {
        return positionMotor.getController().getControlMode() == ControlMode.POSITION || 
        positionMotor.getController().getControlMode() == ControlMode.PROFILED_POSITION;
    }

}
