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

    boolean isrRollerMotorAtSetPoint();

    void stopIsrRollerMotor();

    void setIsrRollerMotorDutyCycle(double dutyCycle);

    boolean isrPositionMotorAtSetPoint();

    void moveToPosition(double positionMeters);


    void updateInputs(IsrIntakeInputs inputs);



}
