package frc.robot.subsystems.intake;

import io.github.captainsoccer.basicmotor.controllers.Controller.ControlMode;
import io.github.captainsoccer.basicmotor.sim.motor.BasicMotorSim;

public class IntakeIOSim implements IntakeIO {
    private final BasicMotorSim intakeMotor;
    private final BasicMotorSim positionMotor;

    public IntakeIOSim() {

        intakeMotor = new BasicMotorSim(IntakeConstants.intakeMotorConfig);

        positionMotor = new BasicMotorSim(IntakeConstants.positionMotorConfig);

    }


    private boolean intakeMotorAtSetPoint() {
        return intakeMotor.atSetpoint();
    }



    @Override
    public void stopIntakeMotor() {
        intakeMotor.stop();
    }

    @Override
    public void setIntakeMotorPercent(double dutyCycle){
        intakeMotor.setPercentOutput(dutyCycle);
    }

    private boolean positionMotorAtSetPoint() {
        return positionMotor.atSetpoint();
    }

    @Override
    public void setPositionMotor(double positionMeters) {
        positionMotor.setControl(positionMeters, ControlMode.POSITION);
    }

    @Override
    public void updateInputs(IntakeInputs inputs) {
        inputs.intakePositionMeters = positionMotor.getPosition();
        inputs.isIntakeMotorAtSetPoint = intakeMotorAtSetPoint();
        inputs.intakeMotorVelocityMPS = intakeMotor.getVelocity();
        inputs.isPositionMotorAtSetPoint = positionMotorAtSetPoint();
        inputs.positionMotorVelocityMPS = getSpeedPositionMotor();
    }

    @Override
    public void stopPositiongMotor() {
        positionMotor.stop();
    }

    private double getSpeedPositionMotor() {
        return positionMotor.getVelocity();
    }

    @Override
    public void setPositionMotorPercent(double dutyCycle) {
        positionMotor.setPercentOutput(dutyCycle);
    }

    @Override
    public void setPositionMotorSlowly(double posMeters){
        positionMotor.setControl(posMeters,ControlMode.PROFILED_POSITION, 1);
    }

    @Override
    public void resetPositionMotor(double posMeters) {
        positionMotor.resetEncoder(posMeters);
    }
}
