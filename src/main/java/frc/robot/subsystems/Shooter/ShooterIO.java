package frc.robot.subsystems.Shooter;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;

public interface ShooterIO {

    @AutoLog
    class ShooterInputs{
       public double shooterSpeedMPS;
       public boolean isFlywheelAtGoal;
       public boolean isKickerStuck;
       public boolean isHoodAtSetpoint;
       public double shootSpeedError;
    }

    /**
     * 
     * @param speedMPS speed to spin up the flywheel to
     */
    void spinUp(double speedMPS);

    /**
     * Keep the current velocity
     */
    void keepVelocity(double speedMPS);

    /**
     * Stops the flywheel
     */
    void stopFlyWheel();

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
     * @param inputs advantage kit inputs object to update
     */
    void update(ShooterInputs inputs);

    void logSysID();

    public class ShooterIOReplay implements ShooterIO{

        @Override
        public void spinUp(double speedMPS) {

        }

        @Override
        public void keepVelocity(double speedMPS) {

        }

        @Override
        public void stopFlyWheel() {

        }

        @Override
        public void setFlyWheelVoltage(double voltage) {

        }

        @Override
        public void toggleKicker(boolean isActive) {

        }

        @Override
        public void setHoodAngle(Rotation2d angle) {

        }

        @Override
        public void update(ShooterInputs inputs) {

        }

        @Override
        public void logSysID() {

        }

    }
}
