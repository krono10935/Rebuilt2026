package frc.robot.subsystems.intake;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase{
  private final IntakeIO io;
  private final IntakeInputsAutoLogged inputs = new IntakeInputsAutoLogged();

    public Intake() {
        io = RobotBase.isReal() ? new IntakeIOSpark() : new IntakeIOSim();
    }

    /**
     * Gives the intake roller a speed in MPS
     * @param velocity the speed MPS
     */
    public void setIntakeMotorVelocity(double velocity){
        io.setIntakeMotorVelocity(velocity);
    }

    /**
     * The position of the intake opening motor in meters
     * @return
     */
    public double getIntakePosition(){
        return io.getIntakePosition();
    }

    /**
     * Applies to the opening motor a duty cycle
     * @param dutyCycle the duty cycle to apply 
     */
    public void setPositionMotorPercent(double dutyCycle){
      io.setPositionMotorPercent(dutyCycle);
    }

    /**
     * @return Whether or not the intake is open
     */
    public boolean isOpen(){
        return getIntakePosition() >= IntakeConstants.OPEN_POSITION - IntakeConstants.POSITION_TOLERANCE;
    }

    /**
     * Tells the intake to open to a certain position
     * @param pos the position in meters
     */
    public void setPosition(double pos){
        io.setPositionMotor(pos);
    }

    /**
     * Applies a duty cycle to the intake roller
     * @param dutyCycle the duty cycle to apply
     */
    public void setPercent(double dutyCycle){
        io.setIntakeMotorPercent(dutyCycle);
    }

    /**
     * Stops the intake roller
     */
    public void stopIntakeMotor(){
        io.stopIntakeMotor();
    }

    /**
     * @return Whether or not the opening motor is at its setpoint (is it fully closed / open)
     */
    public boolean positionAtSetPoint(){
        return io.positionMotorAtSetPoint();
    }

    /**
     * Stop the opening motor
     */
    public void stopIntakeOpeningMotor(){
        io.stopPositiongMotor();
    }

    /**
     * @return whether or not the intake is moving
     */
    private boolean isMoving(){
        boolean isPositionControl = io.isInPositionControl();
        boolean stoppedInPositionControl = isPositionControl && io.positionMotorAtSetPoint();
        double intakeMotorSpeedMPS = io.getSpeedPositionMotor().getRotations() * IntakeConstants.positionMotorConfig.motorConfig.unitConversion;
        boolean isAtZeroSpeed = Math.abs(intakeMotorSpeedMPS) <  IntakeConstants.positionMotorConfig.slot0Config.pidConfig.tolerance;
        boolean stoppedInVelocityControl = !isPositionControl && isAtZeroSpeed;
        boolean isStopped = stoppedInPositionControl || stoppedInVelocityControl;

        return !isStopped;
    }

    /**
     * Tell the position motor to get to a certain position slowly
     * @param posMeters the position to get to
     */
    public void setPositionMotorSlowly(double posMeters){
      io.setPositionMotorSlowly(posMeters);
    }

    /**
     * Reset the opening motor encoder value to a value 
     * @param value the value to reset to
     */
    public void resetEncoderOpen(double value){
      io.resetPositionMotor(value);
    }

    /**
     * @return the current postion of the postion motor in meters
     */
    public double getPositionMotorCurrent() {
        return inputs.positionMotorCurrent;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs(getName(), inputs);
        
        SmartDashboard.putBoolean("Is intake open", isOpen());
        SmartDashboard.putBoolean("Is intake moving", isMoving());

        String currCommand = getCurrentCommand() == null ? "None" : getCurrentCommand().getName();
        Logger.recordOutput("Intake/Current Command ", currCommand);
    }
}
