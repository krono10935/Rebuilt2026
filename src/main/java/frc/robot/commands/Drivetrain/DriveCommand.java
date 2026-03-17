package frc.robot.commands.Drivetrain;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.configsStructure.ChassisConstants;

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
public class DriveCommand extends Command {

    public enum ControllerMode {
        NONE(1),
        BRAKE_EXPONENTIAL(0.5),
        STICKS_EXPONENTIAL(0.5),
        BOTH_EXPONENTIAL(0.5);

        public final double exponent;

        ControllerMode(double exponent) {
            this.exponent = exponent;
        }

        public double calculateTrigger(double value){
            if(this == STICKS_EXPONENTIAL) return value;

            return calculateExponential(value, exponent);
        }

        public double calculateStick(double value){
            if(this == BRAKE_EXPONENTIAL) return value;

            return  calculateExponential(value, exponent);
        }

        private static double calculateExponential(double value, double exponential) {
            return Math.copySign(Math.pow(Math.abs(value), exponential), value);
        }
    }

    protected final Drivetrain drivetrain;
    protected final CommandXboxController controller;

    protected static double MAX_LINEAR_SPEED;
    protected static double MIN_LINEAR_SPEED;
    protected static double MAX_ANGULAR_SPEED;
    protected static double MIN_ANGULAR_SPEED;

    /**
     * Deadband threshold for controller inputs
     */
    protected static final double DEADBAND = 0.1;

    private static final ControllerMode CONTROLLER_MODE = ControllerMode.BOTH_EXPONENTIAL;

    public DriveCommand(Drivetrain drivetrain, CommandXboxController controller) {
        this.drivetrain = drivetrain;
        this.controller = controller;

        addRequirements(drivetrain);

        MAX_LINEAR_SPEED = drivetrain.getConstants().SPEED_CONFIG.maxLinearSpeed();
        MIN_LINEAR_SPEED = drivetrain.getConstants().SPEED_CONFIG.minLinearSpeed();
        MAX_ANGULAR_SPEED = drivetrain.getConstants().MAX_ANGULAR_SPEED;
        MIN_ANGULAR_SPEED = drivetrain.getConstants().MIN_ANGULAR_SPEED;
    }

    /**
     * Calculates the field-relative reference angle.
     */
    protected Rotation2d angleFieldRelative() {
        return ChassisConstants.shouldFlipPath()
                ? drivetrain.getGyroAngle()
                : drivetrain.getGyroAngle().rotateBy(Rotation2d.k180deg);
    }

    @Override
    public void execute() {
        ChassisSpeeds speeds = getControllerInputs();

        drivetrain.drive(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                        speeds.vxMetersPerSecond,
                        speeds.vyMetersPerSecond,
                        speeds.omegaRadiansPerSecond,
                        angleFieldRelative()
                )
        );
    }

    /**
     * Linear interpolation for speed scaling.
     *
     * @param value normalized input (0–1)
     * @return interpolated linear speed
     */
    private static double interpolate(double value) {
        return MIN_LINEAR_SPEED + (MAX_LINEAR_SPEED - MIN_LINEAR_SPEED) * value;
    }

    /**
     * Angular interpolation for rotation scaling.
     *
     * @param value normalized input (0–1)
     * @return interpolated angular speed
     */
    private static double angularInterpolate(double value) {
        return MIN_ANGULAR_SPEED + (MAX_ANGULAR_SPEED - MIN_ANGULAR_SPEED) * value;
    }

    /**
     * Applies deadband to joystick input.
     */
    protected static double deadband(double value) {
        return Math.abs(value) < DEADBAND ? 0 : value;
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.stop();
    }

    /**
     * Converts controller input into chassis speeds.
     */
    public ChassisSpeeds getControllerInputs() {

        double triggerValue = CONTROLLER_MODE.calculateTrigger(1 - controller.getRightTriggerAxis());

        double speed = interpolate(1 - triggerValue);
        double angularSpeed = angularInterpolate(1 - triggerValue);

        double xSpeed = deadband(CONTROLLER_MODE.calculateStick(controller.getLeftY())) * speed;
        double ySpeed = deadband(CONTROLLER_MODE.calculateStick(controller.getLeftX())) * speed;
        double thetaSpeed = deadband(CONTROLLER_MODE.calculateStick(-controller.getRightX())) * angularSpeed;

        return new ChassisSpeeds(xSpeed, ySpeed, thetaSpeed);
    }
}