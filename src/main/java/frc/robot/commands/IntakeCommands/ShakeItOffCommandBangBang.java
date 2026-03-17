package frc.robot.commands.IntakeCommands;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake;

/**
 * Command that repeatedly opens and closes the intake in a bang-bang style.
 * Uses a fixed duty cycle for opening and closes once it reaches target positions.
 */
public class ShakeItOffCommandBangBang extends Command {

    private final Intake intake;

    private final LoggedNetworkNumber openPos;
    private final LoggedNetworkNumber tolerance;
    private final LoggedNetworkNumber openingDutyCycle;
    private final LoggedNetworkNumber closePos;
    private final LoggedNetworkNumber openLessMultiplier;
    private final LoggedNetworkNumber intakeSpeed;

    private final Timer timer;
    private final Timer beginTimer;

    private int cycles;

    @AutoLogOutput(key = "ShakeBangBang/shouldOpen")
    private boolean shouldOpen = false;

    @AutoLogOutput(key = "ShakeBangBang/hasOpened")
    private boolean hasOpened = false;

    /**
     * Creates a new ShakeItOffCommandBangBang.
     *
     * @param intake The intake subsystem this command controls.
     */
    public ShakeItOffCommandBangBang(Intake intake) {
        this.intake = intake;
        addRequirements(intake);

        tolerance = new LoggedNetworkNumber("ShakeBangBang/tolerance", 0.05);
        openingDutyCycle = new LoggedNetworkNumber("ShakeBangBang/openDutyCycle", 0.1);
        openPos = new LoggedNetworkNumber("ShakeBangBang/openPos", 0.25);
        closePos = new LoggedNetworkNumber("ShakeBangBang/closePos", 0.00);
        openLessMultiplier = new LoggedNetworkNumber("ShakeBangBang/openLessMultiplier", 0.75);
        intakeSpeed = new LoggedNetworkNumber("ShakeBangBang/intakeDutyCycle", 0.5);

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
    }

    @Override
    public void execute() {
        if (beginTimer.get() > 0.5) {
            double targetPos = shouldOpen
                    ? openPos.getAsDouble() * Math.pow(openLessMultiplier.getAsDouble(), cycles)
                    : closePos.getAsDouble();

            if (Math.abs(intake.getIntakePosition() - targetPos) < tolerance.getAsDouble()) {
                if (!shouldOpen && hasOpened) {
                    cycles++;
                }
                shouldOpen = !shouldOpen;
                intake.setPositionMotorPercent(openingDutyCycle.getAsDouble());
                timer.reset();
            }
        }
    }
}