package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

/**
 * Command to slowly close the intake by controlling its motor percentage.
 * Gradually reduces the open position and moves the intake motor slowly.
 */
public class SlowlyClose extends Command {

    protected final Intake intake;

    private final LoggedNetworkNumber slowlyClosePercent;


    protected double openPos = IntakeConstants.OPEN_POSITION;

    private boolean closing = true;
    private final Timer timerToOpenAgain;

    /**
     * Creates a new SlowlyClose command.
     *
     * @param intake The intake subsystem to control.
     */
    public SlowlyClose(Intake intake) {
        this.intake = intake;
        this.slowlyClosePercent = new LoggedNetworkNumber("SlowlyClose/percent", 0);
        this.openPos = IntakeConstants.OPEN_POSITION;
        this.timerToOpenAgain = new Timer();
        openPos = IntakeConstants.OPEN_POSITION;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.setPositionMotorSlowly(0);
        intake.setPercent(-0.2);
    }

    @Override
    public void execute() {

        // Gradually reduce open position and move intake
        if (Math.abs(intake.getIntakePosition()) <= 0.003) {
            openPos /= 2.0;
            intake.setPosition(openPos);
            closing = false;
        }

        if (closing && Math.abs(intake.getIntakePosition() - openPos) <= 0.003) {
            closing = true;
            intake.setPositionMotorSlowly(0);
            timerToOpenAgain.reset();
            timerToOpenAgain.start();
        }
    }

    @Override
    public void end(boolean interrupted) {
        intake.setPositionMotorPercent(0);
    }

    @Override
    public boolean isFinished() {
        return false; // This command never finishes on its own
    }
}