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
     * @return if the intake motor is at the setPoint
     */
    boolean intakeMotorAtSetPoint();

    /**
     * 
     * @param velocity velocity per second
     */
    void setIntakeMotorVelocity(double velocity);

    /**
     * sets the intake motor to a certain percent of power
     * @param dutyCycle the percent (0-1)
     */
    void setIntakeMotorPercent(double dutyCycle);

    /**
     * stops the intake motor
     */
    void stopIntakeMotor();

    /**
     * stops the position motor
     */
    void stopPositiongMotor();

    /**
     * 
     * @return if the position motor is at the setPoint
     */
    boolean positionMotorAtSetPoint();

    /**
     * sets the motor to a certain speed in RPS
     * @param velocity the RPS
     */
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

    /**
     * sets the power of the position motor in percent
     * @param dutyCycle the percent 
     */
    void setPositionMotorPercent(double dutyCycle);

    /**
     * sets the motor to a certain position. the motor will execute slowly
     * @param posMeters the position in meters
     */
    void setPositionMotorSlowly(double posMeters);

    /**
     * 
     * @return the current velocity of the motor
     */
    double getPositionMotorVelocity();
    
    void updateInputs(IntakeInputs inputs);

}
