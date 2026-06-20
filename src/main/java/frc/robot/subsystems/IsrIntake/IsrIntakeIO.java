package frc.robot.subsystems.IsrIntake;


import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;

public interface IsrIntakeIO {


    @AutoLog
    class IsrIntakeInputs{
        Rotation2d positionMotorAngle;
        double rollerMotorVelocityMPS;
        boolean isRollerMotorAtSetPoint;
        boolean isPositionMotorAtSetPoint;
    }

    void stopIsrRollerMotor();

    void setIsrRollerMotorDutyCycle(double dutyCycle);

    void moveToPosition(Rotation2d positionMotorAngle);

    void updateInputs(IsrIntakeInputs inputs);



}
