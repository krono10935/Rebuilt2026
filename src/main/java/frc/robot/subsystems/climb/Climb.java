// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climb;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climb extends SubsystemBase {
  /** Creates a new Climb. */
  private final ClimbIO io;

  private final ClimbInputsAutoLogged inputs;

  public Climb() {
    io = RobotBase.isReal() ? new ClimbIOReal() : new ClimbIOSim();
    inputs = new ClimbInputsAutoLogged();
  }

  @Override
  public void periodic() {

    io.update(inputs);

    Logger.processInputs(getName(), inputs);
    Logger.recordOutput(getName()+"/command", getCurrentCommand() == null ? "none" : getCurrentCommand().getName());
    Logger.recordOutput(getName()+"/isAtSetPoint", isAtSetPoint());
  }

  public ClimbConstants.ClimbState getClimbState(){
    return io.getClimbState();
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
  private boolean isAtSetPoint(){
    return io.isAtSetPoint();
  }

  /**
   * 
   * @return the close command
   */
  public Command closeCommand(){
    return new FunctionalCommand(this::close, ()->{}, (interrupted)->{}, this::isAtSetPoint, this);
  }

  /**
   *
   * @return the open command
   */
  public Command openCommand(){
    return new FunctionalCommand(this::open, ()->{}, (interrupted) -> {}, this::isAtSetPoint, this);
  }

}

