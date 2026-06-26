package frc.robot.subsystems.IsrIntake;

import edu.wpi.first.math.geometry.Rotation2d;
import io.github.captainsoccer.basicmotor.controllers.Controller;
import io.github.captainsoccer.basicmotor.sim.motor.BasicMotorSim;

public class IsrIntakeIOSim implements IsrIntakeIO{
    private final BasicMotorSim isrRollerMotor;
    private final BasicMotorSim isrPositionMotor;

    public IsrIntakeIOSim() {

        isrRollerMotor = new BasicMotorSim(IsrIntakeConstants.isrLeadRollerMotorConfig);

        isrPositionMotor = new BasicMotorSim(IsrIntakeConstants.isrLeadPositionMotorConfig);

    }

    private boolean isrRollerMotorAtSetPoint(){
        return isrRollerMotor.atSetpoint();
    }

    @Override
    public void stopIsrRollerMotor(){
        isrRollerMotor.stop();
    }

    @Override
    public void setIsrRollerMotorDutyCycle(double dutyCycle){
        isrRollerMotor.setPercentOutput(dutyCycle);
    }

    private boolean isrPositionMotorAtSetPoint(){
        return isrPositionMotor.atSetpoint();
    }

    @Override
    public void moveToPosition(Rotation2d positionMotorAngle){
        isrPositionMotor.setControl(positionMotorAngle.getDegrees(),Controller.ControlMode.POSITION);
    }

    @Override
    public void updateInputs(IsrIntakeIO.IsrIntakeInputs isrIntakeInputs){
        isrIntakeInputs.positionMotorAngle = Rotation2d.fromDegrees(isrPositionMotor.getPosition());
        isrIntakeInputs.isRollerMotorAtSetPoint = isrRollerMotorAtSetPoint();
        isrIntakeInputs.rollerMotorVelocityMPS = isrRollerMotor.getVelocity();
        isrIntakeInputs.isPositionMotorAtSetPoint = isrPositionMotorAtSetPoint();

    }

}
