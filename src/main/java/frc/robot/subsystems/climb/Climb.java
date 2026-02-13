// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climb;

import edu.wpi.first.wpilibj2.command.*;
import frc.utils.ErrorMessage;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.RobotBase;
import org.w3c.dom.UserDataHandler;

public class Climb extends SubsystemBase implements ErrorMessage.ErrorSender {
  /** Creates a new Climb. */
  private final ClimbIO io;

  private final ClimbInputsAutoLogged inputs;

  private boolean hasClimbed = false;

  private boolean failedToClose = false;
  public Climb() {
    io = RobotBase.isReal() ? new ClimbIOReal() : new ClimbIOSim();
    inputs = new ClimbInputsAutoLogged();

    failedToClose = false;

    ErrorMessage.create(this,
      "error closing" + this.getName(),
              () -> failedToClose);//TODO make each subsystem have its own error msg
  }

  @Override
  public void periodic() {

    io.update(inputs);

    Logger.processInputs(getName(), inputs);
    Logger.recordOutput(getName()+"/command", getCurrentCommand() == null ? "none" : getCurrentCommand().getName());
    Logger.recordOutput(getName()+"/isAtSetPoint", isAtSetPoint());
  }

  /**
   * closes the climb
   */
  private void close(){
    io.close();
  }


  /**
   * opens the climb
   */
  private void open(){
    io.open();
  }

  /**
   * 
   * @return if the climb is at setPoint
   */
  public boolean isAtSetPoint(){
    return io.isAtSetPoint();
  }


    public boolean getHasClimbed() {
        return hasClimbed;
    }

    public void setHasClimbed(boolean hasClimbed) {
        this.hasClimbed = hasClimbed;
    }

  /**
   * 
   * @return the close command
   */
  public Command closeCommand(){
    return new FunctionalCommand(this::close,
            ()->{},
            (interrupted)->{},
            this::isAtSetPoint,
            this);
  }
  
  /**
   * 
   * @return the open command
   */
  public Command openCommand(){
    return new FunctionalCommand(this::open,
            ()->{},
            (interrupted) -> {},
            this::isAtSetPoint,
            this);

  }

  @Override
  public void send(boolean shouldDisplayError){
      failedToClose = shouldDisplayError;
  }


}

