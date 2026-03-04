// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drivetrain;

import com.pathplanner.lib.path.DriveToPoseConstants;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.subsystems.Shooter.ShotCalculator;
import frc.robot.subsystems.Shooter.ShotCalculator.ShootingParameters;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.configsStructure.ChassisConstants;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;


public class DriveAndHomeCommand extends DriveCommand {


    private final ProfiledPIDController angularController;
    public static final Rotation2d robotAngleTolerance = Rotation2d.fromDegrees(2);
    private final Supplier<Rotation2d> targetAngleSupplier;

    private static final double ANGULAR_DEADBAND = Rotation2d.fromDegrees(1).getRadians(); 

    public DriveAndHomeCommand(Drivetrain drivetrain, CommandXboxController controller) {
        super(drivetrain, controller);

        angularController = Constants.THETA_CONTROLLER;

        SmartDashboard.putData("DriveAndHome/thetaController", angularController);
        MAX_LINEAR_SPEED = 2;

        targetAngleSupplier = () -> ShotCalculator.getInstance().getParameters(drivetrain.getEstimatedPosition(),
                drivetrain.getChassisSpeeds()).robotAngle();
    }

    @Override
    public void initialize(){
        angularController.reset(
            new State(drivetrain.getEstimatedPosition().getRotation().getRadians(),
            drivetrain.getChassisSpeeds().omegaRadiansPerSecond));
    }


    @Override
    public void execute() {
        ChassisSpeeds speeds = getControllerInputs();

        ShootingParameters params = ShotCalculator.getInstance().getParameters(drivetrain.getEstimatedPosition(),
        drivetrain.getChassisSpeeds());

        double thetaSpeed =
                angularController.calculate(
                        drivetrain.getEstimatedPosition().getRotation().getRadians(), params.robotAngle().getRadians());

        if(Math.abs(thetaSpeed) < ANGULAR_DEADBAND) thetaSpeed = 0;

        if(Math.abs(speeds.omegaRadiansPerSecond) >= 0.1)
            thetaSpeed = speeds.omegaRadiansPerSecond;

        drivetrain.drive(ChassisSpeeds.fromFieldRelativeSpeeds(
                speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, thetaSpeed, angleFieldRelative()));

        Logger.recordOutput("ShootCommand/thetaError", angularController.getPositionError());

        Logger.recordOutput("ShootCommand/thetaSetpoint", angularController.getGoal());
        Logger.recordOutput("ShootCommand/thetaMeasurement", drivetrain.getEstimatedPosition().getRotation());
    }

    public double calculateThetaPID(){
        return angularController.calculate(
                drivetrain.getEstimatedPosition().getRotation().getRadians(), targetAngleSupplier.get().getRadians());
    }

    public boolean atTargetAngle(){
        return angularController.atGoal();
    }

    /**
     * Calculate the linear lerp of a {@code value}
     * @param value
     * @return the linear lerp of a {@code value}
     */
    private static double lerp(double value){
        return MIN_LINEAR_SPEED + Math.sqrt(MAX_LINEAR_SPEED - MIN_LINEAR_SPEED) * value;
    }

}