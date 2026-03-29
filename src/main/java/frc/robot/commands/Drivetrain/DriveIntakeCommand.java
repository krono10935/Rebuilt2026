package frc.robot.commands.Drivetrain;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.drivetrain.Drivetrain;

public class DriveIntakeCommand extends DriveCommand {

    public DriveIntakeCommand(Drivetrain drivetrain, CommandXboxController controller) {
        super(drivetrain, controller);

        super.MAX_LINEAR_SPEED /= 2;
        super.MAX_ANGULAR_SPEED /= 2;
    }
}
