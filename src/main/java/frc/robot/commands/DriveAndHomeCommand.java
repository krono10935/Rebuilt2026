// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.path.DriveToPoseConstants;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Shooter.ShotCalculator;
import frc.robot.subsystems.Shooter.ShotCalculator.ShootingParameters;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.configsStructure.ChassisConstants;

import java.util.function.Supplier;


public class DriveAndHomeCommand extends Command {
    private final Drivetrain drivetrain;
    private final CommandXboxController controller;

    private static double MAX_LINEAR_SPEED ;
    private static double MIN_LINEAR_SPEED ;

    private final ProfiledPIDController angularController;
    private final Supplier<Rotation2d> targetAngleSupplier;

    private static final double DEADBAND = 0.1;

    public DriveAndHomeCommand(Drivetrain drivetrain, CommandXboxController controller) {
        this.drivetrain = drivetrain;
        this.controller = controller;
        addRequirements(drivetrain);

        var gains = DriveToPoseConstants.ANGULAR_PID_GAINS;
        angularController = new ProfiledPIDController(
                gains.getP(),gains.getI(),gains.getD(),gains.getConstraints());
        angularController.enableContinuousInput(-Math.PI,Math.PI);

        MAX_LINEAR_SPEED = drivetrain.getConstants().SPEED_CONFIG.maxLinearSpeed();
        MIN_LINEAR_SPEED = drivetrain.getConstants().SPEED_CONFIG.minLinearSpeed();

        targetAngleSupplier = () -> ShotCalculator.getInstance().getParameters(drivetrain.getEstimatedPosition(),
                drivetrain.getChassisSpeeds(),
                ChassisSpeeds.fromFieldRelativeSpeeds(drivetrain.getChassisSpeeds(),
                        drivetrain.getGyroAngle())).robotAngle();
    }

    private Rotation2d angleFieldRelative(){
        return ChassisConstants.shouldFlipPath()?
                drivetrain.getGyroAngle():drivetrain.getGyroAngle().rotateBy(Rotation2d.k180deg) ;
    }


    @Override
    public void execute() {
        double speed = lerp(1 - controller.getRightTriggerAxis());


        double xSpeed = deadband(-controller.getLeftX()) * speed;
        double ySpeed = deadband(controller.getLeftY()) * speed;

        ShootingParameters params = ShotCalculator.getInstance().getParameters(drivetrain.getEstimatedPosition(),
        drivetrain.getChassisSpeeds(), 
        ChassisSpeeds.fromFieldRelativeSpeeds(drivetrain.getChassisSpeeds(), drivetrain.getGyroAngle()));

        double thetaSpeed =
                angularController.calculate(
                        drivetrain.getEstimatedPosition().getRotation().getRadians(), params.robotAngle().getRadians());
        drivetrain.drive(ChassisSpeeds.fromFieldRelativeSpeeds(
                xSpeed, ySpeed, thetaSpeed, angleFieldRelative()));
    }

    public double calculateThetaPID(){
        return angularController.calculate(
                drivetrain.getEstimatedPosition().getRotation().getRadians(), targetAngleSupplier.get().getRadians());
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