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
    public boolean getBeamBrake() {
        return false;
    }

    @Override
    public boolean getLimitSwitch(){
        return false;
    }

    @Override
    public void stopMotor() {
        intakeMotor.stop();
    }

    @Override
    public void setVelocityOutput(Rotation2d pos){
        intakeMotor.setControl(pos.getRotations(), ControlMode.VELOCITY);

    }

    @Override
    public double getPos(){
        return positionMotor.getPosition();
    }

    @Override
    public void setPositionMotor(double pos){
        positionMotor.setControl(pos, ControlMode.POSITION);
    }

    @Override
    public void updateInputs(IntakeInputs inputs) {
        inputs.Angle = Rotation2d.fromRotations(positionMotor.getPosition());
        inputs.power = IntakeConstants.INTAKE_KT 
            * intakeMotor.getSensorData().currentOutput()
            * Units.rotationsPerMinuteToRadiansPerSecond(intakeMotor.getVelocity());
        inputs.velocity = intakeMotor.getVelocity(); 
    }


    @Override
    public boolean intakeAtSetPoint() {
        return intakeMotor.atSetpoint();
    }


    @Override
    public boolean positionAtSetPoint() {
        return positionMotor.atSetpoint();
    }
}
