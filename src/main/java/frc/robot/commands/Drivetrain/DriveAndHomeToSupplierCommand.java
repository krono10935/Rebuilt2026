package frc.robot.commands.Drivetrain;

import com.pathplanner.lib.path.DriveToPoseConstants;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.subsystems.drivetrain.Drivetrain;

import org.littletonrobotics.junction.Logger;

import java.util.function.Supplier;

/**
 * Command that drives the robot while automatically rotating
 * toward a supplied angle using a profiled PID controller.
 *
 * <p>
 * Features:
 * <ul>
 *     <li>Field-relative driving</li>
 *     <li>Dynamic angle targeting via Supplier</li>
 *     <li>Driver override for rotation</li>
 * </ul>
 * </p>
 */
public class DriveAndHomeToSupplierCommand extends DriveCommand {

    private final ProfiledPIDController angularController;
    private final Supplier<Rotation2d> angularSupplier;

    /** Small threshold to eliminate rotation jitter */
    private static final double ANGULAR_DEADBAND =
            Rotation2d.fromDegrees(1).getRadians();

    /**
     * Creates the command.
     *
     * @param drivetrain drivetrain subsystem
     * @param controller driver controller
     * @param angleSupplier supplier providing desired robot angle
     */
    public DriveAndHomeToSupplierCommand(
            Drivetrain drivetrain,
            CommandXboxController controller,
            Supplier<Rotation2d> angleSupplier
    ) {
        super(drivetrain, controller);

        this.angularSupplier = angleSupplier;

        var gains = DriveToPoseConstants.ANGULAR_PID_GAINS;

        angularController = new ProfiledPIDController(
                gains.getP(),
                gains.getI(),
                gains.getD(),
                gains.getConstraints()
        );

        angularController.enableContinuousInput(-Math.PI, Math.PI);
    }

    /**
     * Resets the controller at command start.
     */
    @Override
    public void initialize() {
        resetThetaController();
    }

    /**
     * Main execution loop:
     * <ul>
     *     <li>Reads controller input</li>
     *     <li>Calculates rotation using PID</li>
     *     <li>Applies deadband</li>
     *     <li>Allows manual override</li>
     * </ul>
     */
    @Override
    public void execute() {

        ChassisSpeeds speeds = getControllerInputs();

        double thetaSpeed = calculateThetaPID();

        // Prevent jitter
        if (Math.abs(thetaSpeed) < ANGULAR_DEADBAND) {
            thetaSpeed = 0;
        }

        // Driver override for rotation
        if (Math.abs(speeds.omegaRadiansPerSecond) >= DEADBAND) {
            thetaSpeed = speeds.omegaRadiansPerSecond;
        }

        drivetrain.drive(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                        speeds.vxMetersPerSecond,
                        speeds.vyMetersPerSecond,
                        thetaSpeed,
                        angleFieldRelative()
                )
        );
        String name = "HomeCommand/" + getName();
        Logger.recordOutput(name + "/thetaError", angularController.getPositionError());
        Logger.recordOutput(name + "/thetaSetpoint", angularController.getGoal());
        Logger.recordOutput(name + "/thetaMeasurement",
                drivetrain.getEstimatedPosition().getRotation());
    }

    /**
     * Calculates PID output toward target angle.
     *
     * @return angular velocity output
     */
    public double calculateThetaPID() {
        return angularController.calculate(
                drivetrain.getEstimatedPosition().getRotation().getRadians(),
                angularSupplier.get().getRadians()
        );
    }

    /**
     * Resets the theta controller using current robot state.
     */
    public void resetThetaController() {
        angularController.reset(
                new TrapezoidProfile.State(
                        drivetrain.getEstimatedPosition().getRotation().getRadians(),
                        drivetrain.getChassisSpeeds().omegaRadiansPerSecond
                )
        );
    }
}