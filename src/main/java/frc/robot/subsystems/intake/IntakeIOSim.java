package frc.robot.subsystems.intake;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
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
    public void setIntakeMotorVelocity(Rotation2d velocity) {
        intakeMotor.setControl(velocity.getRotations(), ControlMode.VELOCITY);
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
    public boolean positionMotorAtSetPoint() {
        return positionMotor.atSetpoint();
    }

    @Override
    public void resetPositionMotorEncoder() {
        positionMotor.resetEncoder(0);
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
    public boolean getLimitSwitch() {
        return false; 
    }

    @Override
    public void updateInputs(IntakeInputs inputs) {
        inputs.position = positionMotor.getPosition();

        inputs.power = IntakeConstants.INTAKE_KT 
            * intakeMotor.getSensorData().currentOutput()
            * Units.rotationsPerMinuteToRadiansPerSecond(intakeMotor.getVelocity());
        inputs.velocity = intakeMotor.getVelocity(); 
    }
}
