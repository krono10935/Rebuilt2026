package frc.robot.subsystems.IsrIntake;


import org.littletonrobotics.junction.AutoLog;

public interface IsrIntakeIO {


    @AutoLog
    class IsrIntakeInputs{
        double positionMotorMeters;
        double rollerMotorVelocityMPS;
        boolean isRollerMotorAtSetPoint;
        boolean isPositionMotorAtSetPoint;
    }

    void stopIsrRollerMotor();

    void setIsrRollerMotorDutyCycle(double dutyCycle);

    void moveToPosition(double positionMeters);

    void updateInputs(IsrIntakeInputs inputs);



}
