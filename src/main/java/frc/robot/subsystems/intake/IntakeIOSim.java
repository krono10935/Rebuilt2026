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
    public void stopIntakeRoller() {
        intakeMotor.stop();
    }

    @Override
    public void setRollerDutyCycle(double dutyCycle){
        intakeMotor.setPercentOutput(dutyCycle);
    }

    private boolean positionMotorAtSetPoint() {
        return positionMotor.atSetpoint();
    }

    @Override
    public void moveToPosition(double positionMeters) {
        positionMotor.setControl(positionMeters, ControlMode.POSITION);
    }

    @Override
    public void updateInputs(IntakeInputs inputs) {
        inputs.intakePositionMeters = positionMotor.getPosition();
        inputs.isRollerMotorAtSetPoint = intakeMotorAtSetPoint();
        inputs.rollerMotorVelocityMPS = intakeMotor.getVelocity();
        inputs.isPositionMotorAtSetPoint = positionMotorAtSetPoint();
        inputs.positionMotorVelocityMPS = getSpeedPositionMotor();
    }

    @Override
    public void stopPositionMotor() {
        positionMotor.stop();
    }

    private double getSpeedPositionMotor() {
        return positionMotor.getVelocity();
    }

    @Override
    public void setPositionMotorDutyCycle(double dutyCycle) {
        positionMotor.setPercentOutput(dutyCycle);
    }

    @Override
    public void moveToPositionSlowly(double posMeters){
        positionMotor.setControl(posMeters,ControlMode.PROFILED_POSITION, 1);
    }

    @Override
    public void resetOpeningMotorEncoder(double posMeters) {
        positionMotor.resetEncoder(posMeters);
    }
}
