package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.configsStructure.ChassisConstants;


public class CircleAroundAndDrive extends Command {

    private final Drivetrain drivetrain;
    private final CommandXboxController controller;
    private final Translation2d posToLook;


    private final PIDController angularPID;

    private static double MAX_LINEAR_SPEED ;
    private static double MIN_LINEAR_SPEED ;
    private static double MAX_ANGULAR_SPEED;
    private static double MIN_ANGULAR_SPEED;


    private static final double DEADBAND = 0.1;

    public CircleAroundAndDrive(Drivetrain drivetrain, CommandXboxController controller,
                                Translation2d posToLook) {
        this.drivetrain = drivetrain;
        this.controller = controller;
        this.posToLook = posToLook;


        addRequirements(drivetrain);

        MAX_LINEAR_SPEED = drivetrain.getConstants().SPEED_CONFIG.maxLinearSpeed();
        MIN_LINEAR_SPEED = drivetrain.getConstants().SPEED_CONFIG.minLinearSpeed();
        MAX_ANGULAR_SPEED = drivetrain.getConstants().MAX_ANGULAR_SPEED;
        MIN_ANGULAR_SPEED = drivetrain.getConstants().MIN_ANGULAR_SPEED;

        angularPID = new PIDController(1,0,0);
        angularPID.enableContinuousInput(0,360);
    }

    private Rotation2d angleFieldRelative(){
        return ChassisConstants.shouldFlipPath()?
                drivetrain.getGyroAngle():drivetrain.getGyroAngle().rotateBy(Rotation2d.k180deg) ;
    }


    @Override
    public void execute() {
        double speed = lerp(1 - controller.getRightTriggerAxis());
        double angularSpeed = angularLerp(angularPID.calculate(
                angleFieldRelative().getDegrees(),
                AngleToLook().getDegrees()));


        double xSpeed = deadband(-controller.getLeftX()) * speed;
        double ySpeed = deadband(controller.getLeftY()) * speed;



        drivetrain.drive(ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, angularSpeed, angleFieldRelative()));
    }

    public Rotation2d AngleToLook(){
        Translation2d currentPos = drivetrain.getEstimatedPosition().getTranslation();

        double dx = -(currentPos.getX() - posToLook.getX());
        double dy = (currentPos.getY() - posToLook.getY());

        return Rotation2d.fromRadians(Math.atan2(dy,dx));
    }

    /**
     * Calculate the linear lerp of a {@code value}
     * @param value
     * @return the linear lerp of a {@code value}
     */
    private static double lerp(double value){
        return MIN_LINEAR_SPEED + (MAX_LINEAR_SPEED - MIN_LINEAR_SPEED) * value;
    }

    /**
     * Calculate the angular lerp of a {@code value}
     * @param value
     * @return the angular lerp of a {@code value}
     */
    private static double angularLerp(double value){
        return MIN_ANGULAR_SPEED + (MAX_ANGULAR_SPEED - MIN_ANGULAR_SPEED) * value;
    }

    /**
     * Calculate if the value is passed the deadband value
     * @param value
     * @return 0 if the absolute {@code value} is less than deadband, otherwise {@code value}
     */
    private static double deadband(double value){
        if (Math.abs(value) < DEADBAND){
            return 0;
        }

        return value;
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        drivetrain.stop();
    }
}
