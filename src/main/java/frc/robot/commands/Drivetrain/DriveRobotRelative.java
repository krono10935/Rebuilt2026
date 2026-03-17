package frc.robot.commands.Drivetrain;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.subsystems.drivetrain.Drivetrain;

/**
 * Drive command that controls the robot in robot-relative mode
 * instead of field-relative mode.
 *
 * <p>
 * Uses controller inputs directly without adjusting for robot heading.
 * </p>
 */
public class DriveRobotRelative extends DriveCommand {

    /**
     * Creates a robot-relative drive command.
     *
     * @param drivetrain drivetrain subsystem
     * @param controller driver controller
     */
    public DriveRobotRelative(Drivetrain drivetrain, CommandXboxController controller) {
        super(drivetrain, controller);
    }

    /**
     * Executes robot-relative driving.
     */
    @Override
    public void execute() {
        ChassisSpeeds speeds = getControllerInputs();

        drivetrain.drive(new ChassisSpeeds(
                speeds.vxMetersPerSecond,
                speeds.vyMetersPerSecond,
                speeds.omegaRadiansPerSecond
        ));
    }
}