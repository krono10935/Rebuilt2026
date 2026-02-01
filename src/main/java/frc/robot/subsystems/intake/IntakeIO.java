package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;

public interface IntakeIO {
    @AutoLog
    class IntakeInputs {
        Rotation2d Angle;
        double power ;
        double velocity;
    }

    /**
     * 
     * @return if the intake motor is at the setPoint
     */
    boolean intakeAtSetPoint();
    
    /**
     * 
     * @return if the position motor is at the setPoint
     */
    boolean positionAtSetPoint();

    /**
     * 
     * @return if the beam break is broken
     */
    boolean getBeamBrake();

    /**
     * 
     * @return if limit switch is pressed
     */
    boolean getLimitSwitch();

    /**
     * stops the motor
     */
    void stopMotor();

    /**
     * 
     * @param velocity velocity per second
     */
    void setVelocityOutput(Rotation2d velocity);

    /**
     * 
     * @return position of the motor in meters
     */
    double getPos();

    /**
     * sets the postion of the intake
     * @param pos the current position of the intake motor in meters
     */
    void setPositionMotor(double pos);

    void updateInputs(IntakeInputs inputs);





}
