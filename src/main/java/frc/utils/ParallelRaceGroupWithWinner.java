package frc.utils;

import java.util.Collections;
import java.util.LinkedHashSet;
import java.util.Set;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class ParallelRaceGroupWithWinner extends Command {
  // LinkedHashSet guarantees we iterate over commands in the order they were added
  private final Set<Command> m_commands = new LinkedHashSet<>();
  private boolean m_runWhenDisabled = true;
  private Command m_finished = new InstantCommand();
  private InterruptionBehavior m_interruptBehavior = InterruptionBehavior.kCancelIncoming;

  /**
   * Creates a new ParallelCommandRace. The given commands will be executed simultaneously, and will
   * "race to the finish" - the first command to finish ends the entire command, with all other
   * commands being interrupted.
   *
   * @param commands the commands to include in this composition.
   */
  @SuppressWarnings("this-escape")
  public ParallelRaceGroupWithWinner(Command... commands) {
    addCommands(commands);
  }

  /**
   * Adds the given commands to the group.
   *
   * @param commands Commands to add to the group.
   */
  public final void addCommands(Command... commands) {
    if (m_finished == null) {
      throw new IllegalStateException(
          "Commands cannot be added to a composition while it's running!");
    }

    CommandScheduler.getInstance().registerComposedCommands(commands);

    for (Command command : commands) {
      if (!Collections.disjoint(command.getRequirements(), getRequirements())) {
        throw new IllegalArgumentException(
            "Multiple commands in a parallel composition cannot require the same subsystems");
      }
      m_commands.add(command);
      addRequirements(command.getRequirements());
      m_runWhenDisabled &= command.runsWhenDisabled();
      if (command.getInterruptionBehavior() == InterruptionBehavior.kCancelSelf) {
        m_interruptBehavior = InterruptionBehavior.kCancelSelf;
      }
    }
  }

  @Override
  public final void initialize() {
    m_finished = null;
    for (Command command : m_commands) {
      command.initialize();
    }
  }

  @Override
  public final void execute() {
    for (Command command : m_commands) {
      command.execute();
      if (command.isFinished()) {
        m_finished = command;
      }
    }
  }

  @Override
  public final void end(boolean interrupted) {
    for (Command command : m_commands) {
      command.end(!command.isFinished());
    }
  }

  @Override
  public final boolean isFinished() {
    return m_finished != null;
  }

  @Override
  public boolean runsWhenDisabled() {
    return m_runWhenDisabled;
  }

  @Override
  public InterruptionBehavior getInterruptionBehavior() {
    return m_interruptBehavior;
  }

  /**
   * @return either null if there has been no winner, or the command that won the race
   */
  public Command getWinner(){
    return m_finished;
  }

  /**
   * @param winner the commmand you want to win (same instance) in order to run the following command 
   * @param toRun the command you want to run if the winner is the target
   * @return a command composition that runs toRun only if the winner of this race was winner (same instance)
   */
  public Command andThenOnlyIfWinner(Command winner, Command toRun){
    return this.andThen(toRun.onlyIf(() -> getWinner().equals(winner)));
  }

  /**
   * @param command The base command
   * @param timeoutSeconds The timeout given for the command in seconds
   * @param ifTimedOut The command to run if the timeout happened
   * @return A command composition that runs the base command with a timeout and then if the timeout happens runs the ifTimedOut command
   */
  public static Command andThenOnlyIfTimeout(Command command, double timeoutSeconds, Command ifTimedOut){
    Command timeout = new WaitCommand(timeoutSeconds);
    var race = new ParallelRaceGroupWithWinner(command, timeout);
    return race.andThenOnlyIfWinner(timeout, ifTimedOut);
  }
}
