package frc.robot.commands.IntakeCommands;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake;

/**
 * Command that repeatedly opens and closes the intake to "shake" it.
 * The open/close timing, positions, and speed are configurable via LoggedNetworkNumbers.
 */
public class ShakeItOffCommand extends Command {

    private final Intake intake;

    private final LoggedNetworkNumber openPos;
    private final LoggedNetworkNumber tolerance;
    private final LoggedNetworkNumber closePos;
    private final LoggedNetworkNumber closeLessPercent;
    private final LoggedNetworkNumber timeTochange;
    private final LoggedNetworkNumber intakeSpeed;
  private final LoggedNetworkNumber reversePos;

  private final LoggedNetworkNumber tolerance;

    private final Timer timer;
    private final Timer beginTimer;

    private int cycles;

    @AutoLogOutput(key = "Shake/shouldOpen")
    private boolean shouldOpen = false;

    @AutoLogOutput(key = "Shake/hasOpened")
    private boolean hasOpened = false;

    /**
     * Creates a new ShakeItOffCommand.
     *
     * @param intake The intake subsystem this command controls.
     */
    public ShakeItOffCommand(Intake intake) {
        this.intake = intake;
        addRequirements(intake);

        tolerance = new LoggedNetworkNumber("Shake/tolerance", 0.0025);
        openPos = new LoggedNetworkNumber("Shake/openPos", 0.25);
        closePos = new LoggedNetworkNumber("Shake/closePos", 0.00);
        closeLessPercent = new LoggedNetworkNumber("Shake/closeLessMultiplier", 0.75);
        timeTochange = new LoggedNetworkNumber("Shake/time", 1);
        intakeSpeed = new LoggedNetworkNumber("Shake/intakeDutyCycle", 0.5);

        timer = new Timer();
        beginTimer = new Timer();
        cycles = 0;
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();

        beginTimer.reset();
        beginTimer.start();

        intake.setPercent(intakeSpeed.get());
        cycles = 0;
    }
    tolerance = new LoggedNetworkNumber("Shake/tolerance", 0.0025);
    openPos = new LoggedNetworkNumber("Shake/openPos", 0.25);
    reversePos = new LoggedNetworkNumber("Shake/reversePos", 0.1);
    closePos = new LoggedNetworkNumber("Shake/closePos", 0.00);
    closeLessPercent = new LoggedNetworkNumber("Shake/closeLessMultiplier", 0.75);
    timeTochange = new LoggedNetworkNumber("Shake/time", 1);
    timer = new Timer();
    beginTimer = new Timer();
    intakeSpeed = new LoggedNetworkNumber("Shake/intakeDutyCycle", 0.5);
    cycles = 0;

    @Override
    public void execute() {
        if (beginTimer.get() > 0.5) {
            if (timer.get() >= timeTochange.getAsDouble()) {
                if (!shouldOpen) {
                    cycles++;
                }
                shouldOpen = !shouldOpen;

                double targetPosition = shouldOpen
                        ? openPos.getAsDouble() * Math.pow(closeLessPercent.getAsDouble(), cycles)
                        : closePos.getAsDouble();

                intake.setPosition(targetPosition);
                timer.reset();
            }
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
    timer.reset();
    
    timer.start();

    beginTimer.reset();
    beginTimer.start();

    intake.setPercent(intakeSpeed.get());

    cycles = 0;
    
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if(beginTimer.get()> 0.0){
      if(timer.get()>= timeTochange.getAsDouble()){
        if (!shouldOpen){
          cycles++;
        }
    }

    if (intake.getIntakePosition() <= reversePos.getAsDouble()){
          intake.setPercent(-intakeSpeed.get());
    }
  }

    @Override
    public void end(boolean interrupted) {
        intake.stopIntakeMotor();
    }
}