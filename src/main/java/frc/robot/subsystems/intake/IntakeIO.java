package frc.robot.subsystems.intake;
import org.littletonrobotics.junction.AutoLog;
import edu.wpi.first.math.geometry.Rotation2d;

public interface IntakeIO {
    @AutoLog
    class IntakeInputs {
        double position;
        double velocity;
        double positionMotorCurrent;
        double intakeMotorCurrent;
    }

    /**
     * 
     * @return Whether the intake motor is at the setpoint
     */
    boolean intakeMotorAtSetPoint();

    /**
     * 
     * @param scam Intake roller speed MPS (not really, just sets it to 90% power)
     */
    void setIntake90PercentSpeed(double scam);

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
     * 
     * @return Whether the position motor is at the set point
     */
    boolean positionMotorAtSetPoint();

    /**
     * 
     * @return Position of the motor in meters
     */
    double getIntakePosition();

    /**
     * Sets the postion of the intake
     * @param positionMeters the current position of the intake motor in meters
     */
    void setPositionMotor(double positionMeters);

    /**
     * @return The speed which the open/close motor is spinning
     */
    double getSpeedPositionMotor();

    /**
     * @return Whether position motor is in position control
     */
    boolean isInPositionControl();

    /**
     * @return Whether position motor is in velocity control
     */
    boolean isInVelocityControl();

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
