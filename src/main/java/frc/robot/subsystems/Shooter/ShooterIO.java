package frc.robot.subsystems.Shooter;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;

public interface ShooterIO {

    @AutoLog
    class ShooterInputs{
        
        public Rotation2d hoodAngle;

        public double shooterSpeed; // m/s

        public boolean isKickerActive;
    }

    /**
     * 
     * @param speedMPS speeed to spin up the flywheel to
     */
    void spinUp(double speedMPS);

    /**
     * Keep the current velocity
     */
    void keepVelocity();

    /**
     * stops the flywheel
     */
    void stopFlyWheel();

    /**
     * 
     * @return whether or not the shooter is at it's setpoint
     */
    boolean isShooterAtSetpoint();

    /**
     * 
     * @param voltage to apply to the flywheel motor(s)
     */
    void setFlyWheelVoltage(double voltage);

    /**
     * 
     * @param isActive is the kicker active
     */
    void toggleKicker(boolean isActive);


    /**
     * 
     * @param angle the angle to set the hood to
     */
    void setHoodAngle(Rotation2d angle);

    /**
     * 
     * @return whether or not the hood is at it's setpoint
     */
    boolean isHoodAtSetpoint();

    /**
     * 
     * @param inputs advantage kit inputs object to update
     */
    void update(ShooterInputs inputs);
}
