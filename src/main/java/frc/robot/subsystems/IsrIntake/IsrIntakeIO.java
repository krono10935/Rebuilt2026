package frc.robot.subsystems.IsrIntake;


import org.littletonrobotics.junction.AutoLog;

public interface IsrIntakeIO {


    @AutoLog
    class IsrIntakeInputs{
        double positionMotorMeters;
        double rollerMotorVelocityMPS;
        double positionMotorVelocityMPS;
        double positionMotorPositionMPS;
        boolean isRollerMotorAtSetPoint;
        boolean isPositionMotorAtSetPoint;
    }

    void setRollerMotorDutyCycle(double dutyCycle);

    void stopRollerMotor();

    void stopPositionMotor();

    void moveToPositions(double positionMeters);

    void updateInputs(IsrIntakeInputs inputs);

}
