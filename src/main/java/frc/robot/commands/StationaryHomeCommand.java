// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.path.DriveToPoseConstants;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter.ShotCalculator;
import frc.robot.subsystems.Shooter.ShotCalculator.ShootingParameters;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.configsStructure.ChassisConstants;


public class StationaryHomeCommand extends Command {
    private final Drivetrain drivetrain;

    private final ProfiledPIDController angularController;

    public StationaryHomeCommand(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
        addRequirements(drivetrain);

        var gains = DriveToPoseConstants.ANGULAR_PID_GAINS;
        angularController = new ProfiledPIDController(
                gains.getP(),gains.getI(),gains.getD(),gains.getConstraints());
        angularController.enableContinuousInput(-Math.PI,Math.PI);
    }

    private Rotation2d angleFieldRelative(){
        return ChassisConstants.shouldFlipPath()?
                drivetrain.getGyroAngle():drivetrain.getGyroAngle().rotateBy(Rotation2d.k180deg) ;
    }


    @Override
    public void execute() {

        ShootingParameters params = ShotCalculator.getInstance().getParameters(drivetrain.getEstimatedPosition(),
                drivetrain.getChassisSpeeds());

        double thetaSpeed =
                angularController.calculate(
                        drivetrain.getEstimatedPosition().getRotation().getRadians(), params.robotAngle().getRadians());


        drivetrain.drive(ChassisSpeeds.fromFieldRelativeSpeeds(
                0, 0, thetaSpeed, angleFieldRelative()));
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        drivetrain.stop();
    }
}