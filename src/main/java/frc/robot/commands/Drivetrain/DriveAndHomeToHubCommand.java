package frc.robot.commands.Drivetrain;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.Constants;
import frc.robot.subsystems.Shooter.ShotCalculator;
import frc.robot.subsystems.Shooter.ShotCalculator.ShootingParameters;
import frc.robot.subsystems.drivetrain.Drivetrain;

import org.littletonrobotics.junction.Logger;

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

    private final ProfiledPIDController angularController;

    /** Allowed angular error for being "on target". */
    public static final Rotation2d robotAngleTolerance = Rotation2d.fromDegrees(2);

    /** Small threshold below which rotation is ignored. */
    private static final double ANGULAR_DEADBAND = Rotation2d.fromDegrees(1).getRadians();

    /**
     * Creates a new DriveAndHomeCommand.
     *
     * @param drivetrain drivetrain subsystem
     * @param controller driver controller
     */
    public DriveAndHomeToHubCommand(Drivetrain drivetrain, CommandXboxController controller) {
        super(drivetrain, controller, () -> ShotCalculator.getInstance().getParameters(drivetrain.getEstimatedPosition(),
                drivetrain.getChassisSpeeds()).robotAngle());

        angularController = Constants.THETA_CONTROLLER;

        SmartDashboard.putData("DriveAndHome/thetaController", angularController);

        MAX_LINEAR_SPEED = 2;
    }

    /**
     * Resets the angular controller with current robot state.
     */
    @Override
    public void initialize() {  resetThetaController(); }

    /**
     * Main execution loop:
     * <ul>
     *     <li>Reads driver input</li>
     *     <li>Calculates target shooting angle</li>
     *     <li>Applies PID to rotate robot</li>
     *     <li>Allows manual override</li>
     * </ul>
     */
    @Override
    public void execute() {

        ChassisSpeeds speeds = getControllerInputs();

        ShootingParameters params =
                ShotCalculator.getInstance().getParameters(
                        drivetrain.getEstimatedPosition(),
                        drivetrain.getChassisSpeeds()
                );

        double thetaSpeed = angularController.calculate(
                drivetrain.getEstimatedPosition().getRotation().getRadians(),
                params.robotAngle().getRadians()
        );

        // Deadband to prevent jitter
        if (Math.abs(thetaSpeed) < ANGULAR_DEADBAND) {
            thetaSpeed = 0;
        }

        // Driver override for rotation
        if (Math.abs(speeds.omegaRadiansPerSecond) >= 0.1) {
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

        Logger.recordOutput("ShootCommand/thetaError", angularController.getPositionError());
        Logger.recordOutput("ShootCommand/thetaSetpoint", angularController.getGoal());
        Logger.recordOutput("ShootCommand/thetaMeasurement",
                drivetrain.getEstimatedPosition().getRotation());
    }


    /**
     * Resets the theta controller.
     */
    public void resetThetaController() {
        angularController.reset(
                new State(
                        drivetrain.getEstimatedPosition().getRotation().getRadians(),
                        drivetrain.getChassisSpeeds().omegaRadiansPerSecond
                )
        );
    }

}