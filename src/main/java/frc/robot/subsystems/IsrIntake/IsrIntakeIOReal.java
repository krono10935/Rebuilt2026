package frc.robot.subsystems.IsrIntake;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import io.github.captainsoccer.basicmotor.BasicMotor;
import io.github.captainsoccer.basicmotor.controllers.Controller;
import io.github.captainsoccer.basicmotor.rev.BasicSparkFlex;
import io.github.captainsoccer.basicmotor.rev.BasicSparkMAX;

public class IsrIntakeIOReal implements IsrIntakeIO {
    private final BasicMotor isrLeadRollerMotor;
    private final BasicMotor isrLeadPositionMotor;

    private final BasicMotor isrFollowRollerMotor;
    private final BasicMotor isrFollowPositionMotor;

    public IsrIntakeIOReal(){

        isrLeadRollerMotor = new BasicSparkFlex(IsrIntakeConstants.isrLeadRollerMotorConfig);
        isrFollowRollerMotor = new BasicSparkFlex(IsrIntakeConstants.isrFollowRollerMotorConfig);

        isrFollowRollerMotor.followMotor(isrLeadRollerMotor, true);

        SmartDashboard.putData(isrLeadRollerMotor.getController());

        isrLeadPositionMotor = new BasicSparkMAX(IsrIntakeConstants.isrLeadPositionMotorConfig);
        isrFollowPositionMotor = new BasicSparkFlex(IsrIntakeConstants.isrFollowPositionMotorConfig);

        isrFollowPositionMotor.followMotor(isrLeadRollerMotor, true);



    }

    private boolean isrRollerMotorAtSetPoint (){
        return isrLeadRollerMotor.atSetpoint();
    }

    @Override
    public void stopIsrRollerMotor(){
        isrLeadRollerMotor.stop();
    }

    @Override
    public void setIsrRollerMotorDutyCycle(double dutyCycle){
        isrLeadRollerMotor.setPercentOutput(dutyCycle);
    }

    private boolean isrPositionMotorAtSetPoint(){
        return isrLeadPositionMotor.atSetpoint();
    }

    @Override
    public void moveToPosition(Rotation2d positionMotorAngle){
        isrLeadPositionMotor.setControl(positionMotorAngle.getDegrees(), Controller.ControlMode.POSITION);
    }

    @Override
    public void updateInputs(IsrIntakeIO.IsrIntakeInputs isrIntakeInputs){
        isrIntakeInputs.positionMotorAngle =  Rotation2d.fromDegrees(isrLeadPositionMotor.getPosition());
        isrIntakeInputs.isRollerMotorAtSetPoint = isrRollerMotorAtSetPoint();
        isrIntakeInputs.rollerMotorVelocityMPS = isrLeadRollerMotor.getVelocity();
        isrIntakeInputs.isPositionMotorAtSetPoint = isrPositionMotorAtSetPoint();
    }
}
