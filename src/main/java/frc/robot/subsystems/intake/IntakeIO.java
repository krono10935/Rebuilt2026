package frc.robot.subsystems.intake;
import org.littletonrobotics.junction.AutoLog;
import edu.wpi.first.math.geometry.Rotation2d;

public interface IntakeIO {
    @AutoLog
    class IntakeInputs {
        double position;
        double velocity;
        double positionMotorCurrent;
    }

    /**
     * 
     * @return Whether the intake motor is at the setpoint
     */
    boolean intakeMotorAtSetPoint();

    /**
     * 
     * @param velocity Intake roller speed MPS
     */
    void setIntakeMotorVelocity(double velocity);

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
     * Sets the motor to a certain speed in RPS
     * @param velocity the RPS
     */
    void setPositionMotorVelocity(Rotation2d velocity);

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
    Rotation2d getSpeedPositionMotor();

    /**
     * @return If position motor is in position control
     */
    boolean isInPositionControl();

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

    /**
     * 
     * @return The current velocity of the motor
     */
    double getPositionMotorVelocity();
    
    void updateInputs(IntakeInputs inputs);

}
