package frc.robot.subsystems.IsrIntake;

import io.github.captainsoccer.basicmotor.BasicMotor;
import io.github.captainsoccer.basicmotor.controllers.Controller;
import io.github.captainsoccer.basicmotor.rev.BasicSparkFlex;
import io.github.captainsoccer.basicmotor.rev.BasicSparkMAX;

public class IsrIntakeIOReal implements IsrIntakeIO {
    private final BasicMotor isrRollerMotor;
    private final BasicMotor isrPositionMotor;

    public IsrIntakeIOReal(){

        isrRollerMotor = new BasicSparkFlex(IsrIntakeConstants.isrRollerMotorConfig);

        isrPositionMotor = new BasicSparkMAX(IsrIntakeConstants.isrPositionMotorConfig);

    }

    private boolean isrRollerMotorAtSetPoint (){
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
    public void moveToPosition(double positionMeters){
        isrPositionMotor.setControl(positionMeters, Controller.ControlMode.POSITION);
    }

    @Override
    public void updateInputs(IsrIntakeIO.IsrIntakeInputs isrIntakeInputs){
        isrIntakeInputs.positionMotorMeters =  isrPositionMotor.getPosition();
        isrIntakeInputs.isRollerMotorAtSetPoint = isrRollerMotorAtSetPoint();
        isrIntakeInputs.rollerMotorVelocityMPS = isrRollerMotor.getVelocity();
        isrIntakeInputs.isPositionMotorAtSetPoint = isrPositionMotorAtSetPoint();
    }
}
