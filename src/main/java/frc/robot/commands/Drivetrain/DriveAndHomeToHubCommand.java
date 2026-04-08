package frc.robot.commands.Drivetrain;

import java.util.regex.Pattern;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.leds.LED;
import frc.robot.leds.PatternFactory;
import frc.robot.leds.LEDConstants.Segments;
import frc.robot.subsystems.Shooter.ShotCalculator;
import frc.robot.subsystems.drivetrain.Drivetrain;

/**
 * Command that extends {@link DriveCommand} and adds automatic heading alignment
 * using a profiled PID controller.
 *
 * <p>
 * The robot will:
 * <ul>
 *     <li>Drive normally using controller input</li>
 *     <li>Automatically rotate toward a calculated shooting angle</li>
 *     <li>Allow driver override for rotation</li>
 * </ul>
 * </p>
 */
public class DriveAndHomeToHubCommand extends DriveAndHomeToSupplierCommand {
    /** Allowed angular error for being "on target". */
    public static final Rotation2d robotAngleTolerance = Rotation2d.fromDegrees(5);

    /**
     * Creates a new DriveAndHomeCommand.
     *
     * @param drivetrain drivetrain subsystem
     * @param controller driver controller
     */
    public DriveAndHomeToHubCommand(Drivetrain drivetrain, CommandXboxController controller) {
        super(drivetrain, controller, () -> ShotCalculator.getInstance().getParameters(drivetrain.getEstimatedPosition(),
                drivetrain.getChassisSpeeds()).robotAngle());
    }

    @Override
    public void initialize(){
        super.initialize();
        LED.getInstance().setPattern(Segments.RIO, PatternFactory.thetaDriveIndicator(true));
        LED.getInstance().setPattern(Segments.PDH_LEFT, PatternFactory.thetaDriveIndicator(true));
        LED.getInstance().setPattern(Segments.PDH_RIGHT, PatternFactory.thetaDriveIndicator(true));
    }

    @Override
    public void end(boolean interrupted){
        super.end(interrupted);
        LED.getInstance().setPattern(Segments.RIO, PatternFactory.thetaDriveIndicator(false));
        LED.getInstance().setPattern(Segments.PDH_LEFT, PatternFactory.thetaDriveIndicator(false));
        LED.getInstance().setPattern(Segments.PDH_RIGHT, PatternFactory.thetaDriveIndicator(false));
    }
}