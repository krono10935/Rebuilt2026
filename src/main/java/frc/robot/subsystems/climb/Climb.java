// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climb;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.climb.ClimbConstants.ClimbState;
import frc.utils.ErrorMessage;
import frc.utils.ParallelRaceGroupWithWinner;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Alert.AlertType;


public class Climb extends SubsystemBase{
  /** Creates a new Climb. */
  private final ClimbIO io;

  private final ClimbInputsAutoLogged inputs;

  private boolean hasClimbed;

  public Climb() {
    io = RobotBase.isReal() ? new ClimbIOReal() : new ClimbIOSim();
    inputs = new ClimbInputsAutoLogged();

    hasClimbed = false;
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

  private void stop(){
    io.stop();
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
    @SuppressWarnings("resource")
    Alert failedToClose = new Alert("Failed to close climb!", AlertType.kError);

    return new FunctionalCommand(this::close,
            ()->{},
            (interrupted)->io.stop(),
            this::isAtSetPoint,
            this).raceWith(ParallelRaceGroupWithWinner.andThenOnlyIfTimeout(
              new WaitUntilCommand(() -> inputs.state == ClimbState.CLOSED)
                .andThen(new InstantCommand(() -> failedToClose.set(false))),
              ClimbConstants.TIME_FOR_CLIMB_TO_CLOSE_OR_OPEN_CLIMB, 
              new InstantCommand(this::stop)
                .andThen(new InstantCommand(() -> failedToClose.set(true)))));
  }
  
  /**
   * 
   * @return the open command
   */
  public Command openCommand(){
    @SuppressWarnings("resource")
    Alert failedToOpen = new Alert("Failed to open climb!", AlertType.kError);
    
    return new FunctionalCommand(this::open,
            ()->{},
            (interrupted)->io.stop(),
            this::isAtSetPoint,
            this).raceWith(ParallelRaceGroupWithWinner.andThenOnlyIfTimeout(
              new WaitUntilCommand(() -> inputs.state == ClimbState.OPEN)
                .andThen(new InstantCommand(() -> failedToOpen.set(false))),
              ClimbConstants.TIME_FOR_CLIMB_TO_CLOSE_OR_OPEN_CLIMB, 
              new InstantCommand(this::stop)
                .andThen(new InstantCommand(() -> failedToOpen.set(true)))));

  }
}

