package frc.robot.subsystems.climb;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.climb.ClimbConstants.ClimbState;
import frc.utils.ParallelRaceGroupWithWinner;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Alert.AlertType;


public class Climb extends SubsystemBase{
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
    Logger.recordOutput(getName()+"/command",
        getCurrentCommand() == null ? "none" : getCurrentCommand().getName());
    Logger.recordOutput(getName()+"/isAtSetPoint", isAtSetPoint());
  }

  /**
   * Closes the climb
   */
  private void close(){
    io.close();
  }

  /**
   * Opens the climb
   */
  private void open(){
    io.open();
  }

  /**
   * Stop the motor (activate coast)
   */
  private void stop(){
    io.stop();
  }

  /**
   * @return whether the climb is at its setpoint
   */
  public boolean isAtSetPoint(){
    return io.isAtSetPoint();
  }

  /**
   * @return whether or not the robot has climbed
   */
  public boolean getHasClimbed() {
      return hasClimbed;
  }

  /**
   * Set whether or not the robot has climbed
   * @param hasClimbed whether or not the robot has climbed
   */
  public void setHasClimbed(boolean hasClimbed) {
      this.hasClimbed = hasClimbed; //TODO actually use this in our sequences
  }

  /**
   * Builds a command that tries to close the climb, and if it takes too long stop
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
              ClimbConstants.TIME_FOR_CLIMB_TO_CLOSE_OR_OPEN, 
              new InstantCommand(this::stop)
                .andThen(new InstantCommand(() -> failedToClose.set(true)))));
  }
  
  /**
   * Builds a command that tries to open the climb, and if it takes too long stop
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
        ClimbConstants.TIME_FOR_CLIMB_TO_CLOSE_OR_OPEN, 
        new InstantCommand(this::stop)
          .andThen(new InstantCommand(() -> failedToOpen.set(true)))
      )
    );
  }
}

