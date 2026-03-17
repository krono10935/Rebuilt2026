// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase{
  private final IntakeIO io;
  private final IntakeInputsAutoLogged inputs = new IntakeInputsAutoLogged();

  /** Creates a new Intake. */
  public Intake() {

    io = RobotBase.isReal()? new IntakeIOSpark() : new IntakeIOSim();
    }

    public void setIntakeMotorVelocity(double velocity){
        io.setIntakeMotorVelocity(velocity);
    }

    public double getIntakePosition(){
        return io.getIntakePosition();
    }

    public void setPositionMotorPercent(double dutyCycle){
      io.setPositionMotorPercent(dutyCycle);
    }

    public boolean isOpen(){
        return getIntakePosition() >= IntakeConstants.OPEN_POSITION - IntakeConstants.POSITION_TOLERANCE;
    }

    public void setPosition(double pos){
        io.setPositionMotor(pos);
    }

    public void setPercent(double dutyCycle){
        io.setIntakeMotorPercent(dutyCycle);
    }

    public void stopIntakeMotor(){
        io.stopIntakeMotor();
    }

    public boolean positionAtSetPoint(){
        return io.positionMotorAtSetPoint();
    }

    public void setPositionMotorVelocity(Rotation2d velocity){
      io.setPositionMotorVelocity(velocity);
    }

    public void stopIntakeOpeningMotor(){
        io.stopPositiongMotor();
    }

    private boolean isMoving(){
        boolean isPositionControl = io.isInPositionControl();
        boolean stoppedInPositionControl = isPositionControl && io.positionMotorAtSetPoint();
        double intakeMotorSpeedMPS = io.getSpeedPositionMotor().getRotations() * IntakeConstants.positionMotorConfig.motorConfig.unitConversion;
        boolean isAtZeroSpeed = Math.abs(intakeMotorSpeedMPS) <  IntakeConstants.positionMotorConfig.slot0Config.pidConfig.tolerance;
        boolean stoppedInVelocityControl = !isPositionControl && isAtZeroSpeed;
        boolean isStopped = stoppedInPositionControl || stoppedInVelocityControl;

        return !isStopped;
    }

    public void setPositionMotorSlowly(double posMeters){
      io.setPositionMotorSlowly(posMeters);
    }

    public double getPositionMotorVelocity(){
      return io.getPositionMotorVelocity();
    }

    public void resetEncoder(){
        io.resetPositionMotor(0); // arab
    }

    public void resetEncoderOpen(double value){
      io.resetPositionMotor(value);
    }

    public double getPositionMotorCurrent() {
        return inputs.positionMotorCurrent;
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        io.updateInputs(inputs);
        Logger.processInputs(getName(), inputs);
        SmartDashboard.putBoolean("Is intake open", isOpen());
        SmartDashboard.putBoolean("Is intake moving", isMoving());

        String currCommand = getCurrentCommand() == null? "None" : getCurrentCommand().getName();
        Logger.recordOutput("Intake/Current Command ", currCommand);
    }
}
