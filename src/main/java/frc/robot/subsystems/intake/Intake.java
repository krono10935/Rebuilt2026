// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;


import frc.utils.ErrorMessage;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

public class Intake extends SubsystemBase implements ErrorMessage.ErrorSender {
  private final IntakeIO io;
  private final IntakeInputsAutoLogged inputs = new IntakeInputsAutoLogged();
  private int ballsCounter;

  private boolean failedToClose;
  private boolean failedToOpen;
  /** Creates a new Intake. */
  public Intake() {

    if(Robot.isReal()){
      io = new IntakeIOSpark();
    }
    else{
      io = new IntakeIOSim();
    }

    ballsCounter = 0;

    failedToClose = false;

    failedToOpen = false;

    ErrorMessage.create(this,
          "error closing" + this.getName(),
          () -> failedToClose);

    
    ErrorMessage.create(this,
          "error opening" + this.getName(),
          () -> failedToOpen);
    }

    public boolean getLimitSwitch(){
        return io.getLimitSwitch();
    }

    public void setIntakeMotorVelocity(Rotation2d velocity){
        io.setIntakeMotorVelocity(velocity);
    }

    public double getIntakePosition(){
        return io.getIntakePosition();
    }

    public boolean isOpen(){
        return getIntakePosition() >= IntakeConstants.OPEN_POSITION - IntakeConstants.POSITION_TOLERANCE;
    }

    public void setPosition(double pos){
        io.setPositionMotor(pos);
    }

    public void stopIntakeMotor(){
        io.stopIntakeMotor();
    }

    public void setPositionMotorPercentOutput(double percent){
        io.setPositionMotorPercentOutput(percent);
    }

    public double getPower(){
        return inputs.power;
    }

    public boolean intakeMotorAtSetPoint(){
        return io.intakeMotorAtSetPoint();
    }

    public boolean positionAtSetPoint(){
        return io.positionMotorAtSetPoint();
    }

    public void resetPositionMotorEncoder(){
        io.resetPositionMotorEncoder();
    }

    public int getBalls(){
        return ballsCounter;
    }

    public void removeBalls(int decrease){
        ballsCounter -= decrease;
    }

    public void addBalls(int add){
        ballsCounter += add;
    }

    @Override
    public void send(boolean shouldDisplayError, int code){
        switch (code) {
            case 0:
                failedToClose = shouldDisplayError;
            case 1:
                failedToOpen = shouldDisplayError;
            default:
                break;
        }
        
    }

    public void stopIntakeOpeningMotor(){
        io.stopIntakeOpeningMotor();
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        io.updateInputs(inputs);
        Logger.processInputs(getName(), inputs);

        String currCommand = getCurrentCommand() == null? "None" : getCurrentCommand().getName();
        Logger.recordOutput("Intake/Current Command ", currCommand);

    }
}
