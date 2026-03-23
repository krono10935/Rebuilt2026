package frc.robot.commands.IntakeCommands;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.configsStructure.ChassisConstants;
import frc.robot.subsystems.intake.Intake;
import frc.utils.AllianceFlipUtil;

/**
 * Base driving command for the drivetrain.
 *
 * <p>
 * Handles:
 * <ul>
 *     <li>Controller input processing</li>
 *     <li>Speed scaling (linear & angular)</li>
 *     <li>Deadband filtering</li>
 *     <li>Field-relative driving</li>
 * </ul>
 * </p>
 */
public class ManualIntake extends Command {

    private final Intake intake;
    private final CommandXboxController controller;

    private static LoggedNetworkNumber MAX_INTAKE_OPENING_SPEED;
    private static LoggedNetworkNumber MIN_INTAKE_OPENING_SPEED;


    /**
     * Deadband threshold for controller inputs
     */
    protected static final double DEADBAND = 0.1;

    public ManualIntake(Intake intake, CommandXboxController controller) {
        this.intake = intake;
        this.controller = controller;

        MAX_INTAKE_OPENING_SPEED = new LoggedNetworkNumber("ManualIntake/Max speed", 0.9);
        MIN_INTAKE_OPENING_SPEED = new LoggedNetworkNumber("ManualIntake/Min speed", 0.15);

        addRequirements(intake);
    }


    @Override
    public void execute() {
        double speed = getControllerInputs();

        intake.setPercent(speed);
    }

    /**
     * Linear interpolation for speed scaling.
     *
     * @param value normalized input (0–1)
     * @return interpolated linear speed
     */
    private static double interpolate(double value) {
        return MIN_INTAKE_OPENING_SPEED.getAsDouble()
         + (MAX_INTAKE_OPENING_SPEED.getAsDouble() - MIN_INTAKE_OPENING_SPEED.getAsDouble()) * value;
    }

    /**
     * Applies deadband to joystick input.
     */
    protected static double deadband(double value) {
        return Math.abs(value) < DEADBAND ? 0 : value;
    }

    @Override
    public void end(boolean interrupted) {
        intake.stopIntakeOpeningMotor();
    }

    /**
     * Converts controller input into chassis speeds.
     */
    public double getControllerInputs() {

        double speed = interpolate(1 - controller.getRightTriggerAxis());


        double intakeSpeed = deadband(-controller.getLeftY()) * speed;

        return intakeSpeed;
    }
}