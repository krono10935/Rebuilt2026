package frc.robot.subsystems.IsrIntake;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.AbsoluteEncoder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
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

    private final DutyCycleEncoder encoder;

    public IsrIntakeIOReal(){

        isrLeadRollerMotor = new BasicSparkFlex(IsrIntakeConstants.isrLeadRollerMotorConfig);
        isrFollowRollerMotor = new BasicSparkFlex(IsrIntakeConstants.isrFollowRollerMotorConfig);

        isrFollowRollerMotor.followMotor(isrLeadRollerMotor, true);

        isrLeadPositionMotor = new BasicSparkMAX(IsrIntakeConstants.isrLeadPositionMotorConfig);
        isrFollowPositionMotor = new BasicSparkMAX(IsrIntakeConstants.isrFollowPositionMotorConfig);

        SmartDashboard.putData(isrLeadPositionMotor.getController());
        SmartDashboard.putData(isrFollowPositionMotor.getController());


        isrFollowPositionMotor.followMotor(isrLeadPositionMotor, true);

        encoder = new DutyCycleEncoder(IsrIntakeConstants.DUTY_CYCLE_ENCODER_PORT);

        encoder.setInverted(IsrIntakeConstants.DUTY_CYCLE_ENCODER_INVERTED);

        isrLeadPositionMotor.resetEncoder(
            (encoder.get() -
             IsrIntakeConstants.DUTY_CYCLE_ENCODER_OFFSET.getRotations()) *
         IsrIntakeConstants.isrLeadPositionMotorConfig.motorConfig.unitConversion);



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
        isrLeadPositionMotor.setControl(positionMotorAngle.getDegrees(), Controller.ControlMode.PROFILED_POSITION);
    }

    @Override
    public void updateInputs(IsrIntakeIO.IsrIntakeInputs isrIntakeInputs){
        isrIntakeInputs.positionMotorAngle =  Rotation2d.fromDegrees(isrLeadPositionMotor.getPosition());
        isrIntakeInputs.isRollerMotorAtSetPoint = isrRollerMotorAtSetPoint();
        isrIntakeInputs.rollerMotorVelocityMPS = isrLeadRollerMotor.getVelocity();
        isrIntakeInputs.isPositionMotorAtSetPoint = isrPositionMotorAtSetPoint();

        Logger.recordOutput("IsrIntake/DutyCycle Encoder Offset", isrLeadPositionMotor.getPosition() /
         IsrIntakeConstants.isrLeadPositionMotorConfig.motorConfig.unitConversion);

        // isrLeadPositionMotor.resetEncoder(
        //     (encoder.get() - IsrIntakeConstants.DUTY_CYCLE_ENCODER_OFFSET_ROTATIONS) *
        //  IsrIntakeConstants.isrLeadPositionMotorConfig.motorConfig.unitConversion);
    }
}

