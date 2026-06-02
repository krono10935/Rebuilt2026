package frc.robot.subsystems.IsrIntake;

import io.github.captainsoccer.basicmotor.controllers.Controller;
import io.github.captainsoccer.basicmotor.sim.motor.BasicMotorSim;

public class IsrIntakeIOSim {
    private final BasicMotorSim isrRollerMotor;
    private final BasicMotorSim isrPositionMotor;

    public IsrIntakeIOSim() {

        isrRollerMotor = new BasicMotorSim(IsrIntakeConstants.isrRollerMotorConfig);

        isrPositionMotor = new BasicMotorSim(IsrIntakeConstants.isrPositionMotorConfig);

    }

    private boolean isrRollerMotorAtSetPoint(){
        return isrPositionMotor.atSetpoint();
    }

    public void stopIsrRollerMotor(){
        isrRollerMotor.stop();
    }

    public void setIsrRollerMotorDutyCycle(double dutyCycle){
        isrRollerMotor.setPercentOutput(dutyCycle);
    }

    private boolean isrPositionMotorAtSetPoint(){
        return isrPositionMotor.atSetpoint();
    }

    public void moveToPosition(double positionMeters){
        isrPositionMotor.setControl(positionMeters,Controller.ControlMode.POSITION);
    }

    public void updateInputs(IsrIntakeIO.IsrIntakeInputs isrIntakeInputs){
        isrIntakeInputs.positionMotorMeters = isrPositionMotor.getPosition();
        isrIntakeInputs.isRollerMotorAtSetPoint = isrRollerMotorAtSetPoint();
        isrIntakeInputs.rollerMotorVelocityMPS = isrRollerMotor.getVelocity();
        isrIntakeInputs.isPositionMotorAtSetPoint = isrPositionMotorAtSetPoint();
        isrIntakeInputs.positionMotorVelocityMPS = isrPositionMotor.getVelocity();
    }


}
