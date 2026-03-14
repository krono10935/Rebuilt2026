package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;

public interface IntakeIO {
    @AutoLog
    class IntakeInputs {
        double position;
        double velocity;
    }

    /**
     * 
     * @return if the intake motor is at the setPoint
     */
    boolean intakeMotorAtSetPoint();

    /**
     * 
     * @param velocity velocity per second
     */
    void setIntakeMotorVelocity(double velocity);

    void setIntakeMotorPercent(double dutyCycle);
    /**
     * stops the intake motor
     */
    void stopIntakeMotor();

    /**
     * stops the opening intake motor
     */
    void stopIntakeOpeningMotor();

    /**
     * sets the power of the motor in percent
     */
    void setPositionMotorPercentOutput(double percent);

    /**
     * 
     * @return if the position motor is at the setPoint
     */
    boolean positionMotorAtSetPoint();

    void setPositionMotorVelocity(Rotation2d velocity);

    /**
     * 
     * @return position of the motor in meters
     */
    double getIntakePosition();

    /**
     * sets the postion of the intake
     * @param positionMeters the current position of the intake motor in meters
     */
    void setPositionMotor(double positionMeters);

    /**
     * @return the speed which the open/close motor is spinning
     */
    Rotation2d getSpeedPositionMotor();

    /**
     * @return if position motor is in position control
     */
    boolean isInPositionControl();

    /**
     * resets the position to a certain position in units of meters
     */
    void resetPositionMotor(double posMeters);

    void setPositionMotorPercent(double dutyCycle);

    void setPositionMotorSlowly(double posMeters);

    double getIntakePositionMotorVelocity ();

    void updateInputs(IntakeInputs inputs);

}
