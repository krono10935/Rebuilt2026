package frc.robot.subsystems.intake;
import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
    @AutoLog
    class IntakeInputs {
        double intakePositionMeters;
        double rollerMotorVelocityMPS;
        double positionMotorCurrentAmps;
        boolean isRollerMotorAtSetPoint;
        boolean isPositionMotorAtSetPoint;
        double positionMotorVelocityMPS;
    }


    /**
     * Sets the intake motor to a certain percent of power
     * @param dutyCycle the percent (0-1)
     */
    void setRollerDutyCycle(double dutyCycle);

    /**
     * Stops the intake motor
     */
    void stopIntakeRoller();

    /**
     * Stops the position motor
     */
    void stopPositionMotor();

    /**
     * Sets the postion of the intake
     * @param positionMeters the current position of the intake motor in meters
     */
    void moveToPosition(double positionMeters);

    /**
     * Resets the position to a certain position in units of meters
     */
    void resetOpeningMotorEncoder(double posMeters);

    /**
     * Sets the power of the position motor in percent
     * @param dutyCycle the percent 
     */
    void setPositionMotorDutyCycle(double dutyCycle);

    /**
     * Sets the motor to a certain position. the motor will execute slowly
     * @param posMeters the position in meters
     */
    void moveToPositionSlowly(double posMeters);

    void updateInputs(IntakeInputs inputs);

    public class IntakeIOReplay implements IntakeIO{

        public IntakeIOReplay(){
            
        }

        @Override
        public void setRollerDutyCycle(double dutyCycle) {

        }

        @Override
        public void stopIntakeRoller() {

        }

        @Override
        public void stopPositionMotor() {

        }

        @Override
        public void moveToPosition(double positionMeters) {

        }

        @Override
        public void resetOpeningMotorEncoder(double posMeters) {

        }

        @Override
        public void setPositionMotorDutyCycle(double dutyCycle) {

        }

        @Override
        public void moveToPositionSlowly(double posMeters) {

        }

        @Override
        public void updateInputs(IntakeInputs inputs) {

        }
        
    }

}
