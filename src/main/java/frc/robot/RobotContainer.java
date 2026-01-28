// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.SwerveSysID;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.constants.ChassisType;
import io.github.captainsoccer.basicmotor.gains.PIDGains;

import org.littletonrobotics.conduit.ConduitApi;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.DriveToPose;
import com.pathplanner.lib.path.DriveToPoseConstants;
import edu.wpi.first.wpilibj2.command.*;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;


public class RobotContainer
{

    private static RobotContainer instance;

    public final Drivetrain drivetrain;

    private final CommandXboxController controller;

    private final LoggedDashboardChooser<Command> chooser;


    public static RobotContainer getInstance(){
        if (instance == null){
            instance = new RobotContainer();
        }
        return instance;
    }

    private RobotContainer()
    {
        controller = new CommandXboxController(0);

        drivetrain = new Drivetrain(ConduitApi.getInstance()::getPDPVoltage, Constants.CHASSIS_TYPE.constants);

        configureBindings();
        chooser = new LoggedDashboardChooser<>("chooser", AutoBuilder.buildAutoChooser());
    }

    private void configureBindings() {

        drivetrain.setDefaultCommand(new DriveCommand(drivetrain, controller));
        SwerveSysID sysID = new SwerveSysID(drivetrain, controller);
        controller.a().whileTrue(sysID.sysIdDynamicDrive(SysIdRoutine.Direction.kForward));
        controller.b().whileTrue(sysID.sysIdDynamicDrive(SysIdRoutine.Direction.kReverse));
        controller.y().whileTrue(sysID.sysIdQuasistaticDrive(SysIdRoutine.Direction.kForward));
        controller.x().whileTrue(sysID.sysIdQuasistaticDrive(SysIdRoutine.Direction.kReverse));
        controller.rightBumper().onTrue(new InstantCommand(() ->
                drivetrain.reset(new Pose2d(drivetrain.getEstimatedPosition().getTranslation(), new Rotation2d())))
                .ignoringDisable(true));

        DriveToPoseConstants.ANGULAR_PID_GAINS = new ProfiledPIDController(12,25.00,0,
        new TrapezoidProfile.Constraints(3, Units.degreesToRadians(45)));
        DriveToPoseConstants.MAX_LINEAR_SPEED = 3;
        DriveToPoseConstants.ANGLE_TOLERANCE = Units.degreesToRadians(5);
        DriveToPoseConstants.FF_MIN_ANGLE = Units.degreesToRadians(1);
        SmartDashboard.putData(DriveToPoseConstants.ANGULAR_PID_GAINS);
        DriveToPoseConstants.FF_MAX_DISTANCE = 0.15;
        DriveToPoseConstants.FF_MIN_DISTANCE = 0.05;
        SmartDashboard.putData(DriveToPoseConstants.LINEAR_PID_GAINS);
        DriveToPoseConstants.LINEAR_PID_GAINS = new PIDController(10,2,0);
        DriveToPoseConstants.POSE_TOLERANCE = 0.01;
        DriveToPoseConstants.LINEAR_PID_GAINS.setIntegratorRange(
            Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY);
    
    }
    public Command getAutonomousCommand()
    {
        return chooser.get();
    }


}