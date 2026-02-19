// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;


import edu.wpi.first.wpilibj.RobotBase;
import frc.utils.ErrorMessage;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

public class Intake extends SubsystemBase implements ErrorMessage.ErrorSender {
  private final IntakeIO io;
  private final IntakeInputsAutoLogged inputs = new IntakeInputsAutoLogged();

  private boolean failedToClose;
  private boolean failedToOpen;

  @AutoLogOutput
  private boolean hasBalls = false;
  /** Creates a new Intake. */
  public Intake() {

    io = new IntakeIOSim();//RobotBase.isReal()? new IntakeIOSpark() : new IntakeIOSim();

    failedToClose = false;

    failedToOpen = false;

    ErrorMessage.create(this,
          "error closing" + this.getName(),
          () -> failedToClose);

    
    ErrorMessage.create(this,
          "error opening" + this.getName(),
          () -> failedToOpen);
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


    public double getPower(){
        return inputs.power;
    }

    public boolean positionAtSetPoint(){
        return io.positionMotorAtSetPoint();
    }

    public void setPositionMotorVelocity(Rotation2d velocity){
      io.setPositionMotorVelocity(velocity);
    }

    public void setHasBalls(boolean hasBalls){
      this.hasBalls = hasBalls;
    }

    public boolean hasBalls(){
      return hasBalls;
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
