package frc.robot.subsystems.intake;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.intake.IntakeIO.IntakeIOReplay;


public class Intake extends SubsystemBase{
  private final IntakeIO io;
  private final IntakeInputsAutoLogged inputs;

    public Intake() {
        
        switch (Constants.currentMode) {
            case REAL -> io = new IntakeIOSpark();

            case SIM -> io = new IntakeIOSim();
        
            default -> io = new IntakeIOReplay();
        }

        inputs = new IntakeInputsAutoLogged();

    }

    /**
     * @return The position of the intake opening motor in meters
     */
    public double getIntakePosition(){
        return inputs.intakePositionMeters;
    }

    /**
     * Applies to the opening motor a duty cycle
     * @param dutyCycle the duty cycle to apply 
     */
    public void setPositionMotorDutyCycle(double dutyCycle){
      io.setPositionMotorDutyCycle(dutyCycle);
    }

    /**
     * @return Whether or not the intake is open
     */
    public boolean isFullyOpen(){
        return inputs.intakePositionMeters >= IntakeConstants.OPEN_POSITION - IntakeConstants.POSITION_TOLERANCE;
    }

    /**
     * Tells the intake to open to a certain position
     * @param pos the position in meters
     */
    public void moveToPosition(double pos){
        io.moveToPosition(pos);
    }

    /**
     * Applies a duty cycle to the intake roller
     * @param dutyCycle the duty cycle to apply
     */
    public void setRollerDutyCycle(double dutyCycle){
        io.setRollerDutyCycle(dutyCycle);
    }

    /**
     * Stops the intake roller
     */
    public void stopIntakeRoller(){
        io.stopIntakeRoller();
    }

    /**
     * @return Whether or not the opening motor is at its setpoint
     */
    public boolean isPositionAtSetPoint(){
        return inputs.isPositionMotorAtSetPoint;
    }

    /**
     * Stop the opening motor
     */
    public void stopPositionMotor(){
        io.stopPositionMotor();
    }

    /**
     * @return whether or not the intake is moving
     */
    public boolean isMoving(){

        double intakeMotorSpeedMPS = inputs.rollerMotorVelocityMPS;
        boolean isAtZeroSpeed = Math.abs(intakeMotorSpeedMPS) < IntakeConstants.SPEED_DEADBAND;

        return !isAtZeroSpeed;
    }

    /**
     * Tell the position motor to get to a certain position slowly
     * @param posMeters the position to get to
     */
    public void moveToPositionSlowly(double posMeters){
      io.moveToPositionSlowly(posMeters);
    }

    /**
     * Reset the opening motor encoder value to a value 
     * @param value the value to reset to
     */
    public void resetOpeningMotorEncoder(double value){
      io.resetOpeningMotorEncoder(value);
    }

    /**
     * @return the electric current of the postion motor
     */
    public double getPositionMotorCurrentAmps() {
        return inputs.positionMotorCurrentAmps;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs(getName(), inputs);

        String currCommand = getCurrentCommand() == null ? "None" : getCurrentCommand().getName();
        Logger.recordOutput("Intake/Current Command ", currCommand);
    }
}
