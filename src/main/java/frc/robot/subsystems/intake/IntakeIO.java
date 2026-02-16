package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;

public interface IntakeIO {
    @AutoLog
    class IntakeInputs {
        double position;
        double power;
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
    void setIntakeMotorVelocity(Rotation2d velocity);

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

    /**
     * resets the position motor's encoder to 0
     */
    void resetPositionMotorEncoder();
 
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

    void updateInputs(IntakeInputs inputs);

}
