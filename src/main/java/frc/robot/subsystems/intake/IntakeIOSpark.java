package frc.robot.subsystems.intake;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import io.github.captainsoccer.basicmotor.BasicMotor;
import io.github.captainsoccer.basicmotor.controllers.Controller.ControlMode;
import io.github.captainsoccer.basicmotor.rev.BasicSparkMAX;

public class IntakeIOSpark implements IntakeIO {
    private final BasicMotor intakeMotor;
    private final BasicMotor positionMotor;
    private final DigitalInput limitSwitch;

    public IntakeIOSpark() {

        intakeMotor = new BasicSparkMAX(IntakeConstants.intakeMotorConfig);

        positionMotor = new BasicSparkMAX(IntakeConstants.positionMotorConfig);
        
        limitSwitch = new DigitalInput(IntakeConstants.LIMIT_SWITCH_CHANNEL);
        
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
    public void setPositionMotor(double pos) {
        positionMotor.setControl(pos, ControlMode.POSITION);
    }

    @Override
    public boolean getLimitSwitch() {
        return limitSwitch.get();
    }

    @Override
    public void updateInputs(IntakeInputs inputs) {
        inputs.Angle = Rotation2d.fromRotations(positionMotor.getPosition());
        inputs.power = IntakeConstants.INTAKE_KT 
            * intakeMotor.getSensorData().currentOutput()
            * Units.rotationsPerMinuteToRadiansPerSecond(intakeMotor.getVelocity());
        inputs.velocity = intakeMotor.getVelocity(); 
    }

}
