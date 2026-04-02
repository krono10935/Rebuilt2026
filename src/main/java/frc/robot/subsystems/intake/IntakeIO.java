package frc.robot.subsystems.intake;
import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
    @AutoLog
    class IntakeInputs {
        double intakePositionMeters;
        double intakeMotorVelocityMPS;
        double positionMotorCurrentAmps;
        double intakeMotorCurrentAmps;
        boolean isIntakeMotorAtSetPoint;
        boolean isPositionMotorAtSetPoint;
        double positionMotorVelocityMPS;
    }


    /**
     * Sets the intake motor to a certain percent of power
     * @param dutyCycle the percent (0-1)
     */
    void setIntakeMotorPercent(double dutyCycle);

    /**
     * Stops the intake motor
     */
    void stopIntakeMotor();

    /**
     * Stops the position motor
     */
    void stopPositiongMotor();

    /**
     * Sets the postion of the intake
     * @param positionMeters the current position of the intake motor in meters
     */
    void setPositionMotor(double positionMeters);

    /**
     * Resets the position to a certain position in units of meters
     */
    void resetPositionMotor(double posMeters);

    /**
     * Sets the power of the position motor in percent
     * @param dutyCycle the percent 
     */
    void setPositionMotorPercent(double dutyCycle);

    /**
     * Sets the motor to a certain position. the motor will execute slowly
     * @param posMeters the position in meters
     */
    void setPositionMotorSlowly(double posMeters);

    void updateInputs(IntakeInputs inputs);

}
