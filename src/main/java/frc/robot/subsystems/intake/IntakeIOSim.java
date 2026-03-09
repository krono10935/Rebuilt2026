package frc.robot.subsystems.intake;

import edu.wpi.first.math.geometry.Rotation2d;
import io.github.captainsoccer.basicmotor.controllers.Controller.ControlMode;
import io.github.captainsoccer.basicmotor.sim.motor.BasicMotorSim;

public class IntakeIOSim implements IntakeIO {
    private final BasicMotorSim intakeMotor;
    private final BasicMotorSim positionMotor;


    public IntakeIOSim() {

        intakeMotor = new BasicMotorSim(IntakeConstants.intakeMotorConfig);

        positionMotor = new BasicMotorSim(IntakeConstants.positionMotorConfig);

    }


    @Override
    public boolean intakeMotorAtSetPoint() {
        return intakeMotor.atSetpoint();
    }

    @Override
    public void setIntakeMotorVelocity(double velocity) {
        intakeMotor.setControl(velocity, ControlMode.VELOCITY);
    }

    @Override
    public void stopIntakeMotor() {
        intakeMotor.stop();
    }

    @Override
    public void setPositionMotorPercentOutput(double percent){
        positionMotor.setPercentOutput(percent);
    }

    @Override
    public void setIntakeMotorPercent(double dutyCycle){
        intakeMotor.setPercentOutput(dutyCycle);
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
    public void setPositionMotor(double positionMeters) {
        positionMotor.setControl(positionMeters, ControlMode.POSITION);
    }


    @Override
    public void updateInputs(IntakeInputs inputs) {
        inputs.position = positionMotor.getPosition();

        inputs.velocity = intakeMotor.getVelocity(); 
    }


    @Override
    public void stopIntakeOpeningMotor() {
        positionMotor.stop();
    }


    @Override
    public Rotation2d getSpeedPositionMotor() {
        return Rotation2d.fromRotations(positionMotor.getPosition());
    }

    @Override
    public boolean isInPositionControl() {
        return positionMotor.getController().getControlMode() == ControlMode.POSITION || 
        positionMotor.getController().getControlMode() == ControlMode.PROFILED_POSITION;
    }

    @Override
    public void setPositionMotorPercent(double dutyCycle) {
        positionMotor.setPercentOutput(dutyCycle);
    }

    @Override
    public void resetPositionMotor(double posMeters) {
        positionMotor.resetEncoder(posMeters);
    }
}
