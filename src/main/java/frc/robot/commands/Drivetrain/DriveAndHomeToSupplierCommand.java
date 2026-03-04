// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drivetrain;

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


public class DriveAndHomeToSupplierCommand extends DriveCommand {

    private final ProfiledPIDController angularController;

    private final Supplier<Rotation2d> angularSupplier;

    public DriveAndHomeToSupplierCommand(Drivetrain drivetrain, CommandXboxController controller, Supplier<Rotation2d> angleSupplier) {
     
        super(drivetrain, controller);
        this.angularSupplier = angleSupplier;


        var gains = DriveToPoseConstants.ANGULAR_PID_GAINS;
        angularController = new ProfiledPIDController(
                gains.getP(),gains.getI(),gains.getD(),gains.getConstraints());
        angularController.enableContinuousInput(-Math.PI,Math.PI);
    }


    @Override
    public void execute() {
        ChassisSpeeds speeds = getControllerInputs();

        double thetaSpeed =
                angularController.calculate(
                        drivetrain.getEstimatedPosition().getRotation().getRadians(), angularSupplier.get().getRadians());

        if(Math.abs(speeds.omegaRadiansPerSecond) >= 0.1)
            thetaSpeed = speeds.omegaRadiansPerSecond;

        drivetrain.drive(ChassisSpeeds.fromFieldRelativeSpeeds(
                speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, thetaSpeed, angleFieldRelative()));

    }

    public boolean atTargetAngle(){
        return angularController.atGoal();
    }
}