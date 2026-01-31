// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.Shooter.AutoShootAndAim;
import frc.robot.commands.Shooter.KeepVelocity;
import frc.robot.commands.Shooter.SpinUp;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Shooter.ShooterSysID;
import frc.robot.subsystems.drivetrain.Drivetrain;

import org.littletonrobotics.conduit.ConduitApi;

import edu.wpi.first.wpilibj2.command.*;


public class RobotContainer
{

    public static Vision vision;
    private static RobotContainer instance;

    private final Shooter shooter;

    private final CommandXboxController xboxController;

    private final Drivetrain drivetrain;

    // private final LoggedDashboardChooser<Command> chooser;


    public static RobotContainer getInstance(){
        if (instance == null){
            instance = new RobotContainer();
        }
        return instance;
    }

    Supplier<Pose2d>lastPoseSupplier;

    private RobotContainer()
    {
        poseX = new LoggedNetworkNumber("PoseX", 0);
        poseY = new LoggedNetworkNumber("PoseY", 0);

        shooter = new Shooter();

        xboxController = new CommandXboxController(0);

        drivetrain = new Drivetrain(ConduitApi.getInstance()::getPDPVoltage, Constants.CHASSIS_TYPE.constants);

        drivetrain.setDefaultCommand(new DriveCommand(drivetrain, xboxController));

        shooter.setDefaultCommand(new AutoShootAndAim(shooter, drivetrain));
        configureBindings();
        // chooser = new LoggedDashboardChooser<>("chooser", AutoBuilder.buildAutoChooser());
    }

    private void configureBindings() {

        // xboxController.a()
        // .whileTrue(new SpinUp(shooter).andThen(new KeepVelocity(shooter)))
        // .onFalse(new InstantCommand(()-> shooter.stopFlyWheel()).ignoringDisable(true));
    }
    
    
    public Command getAutonomousCommand()
    {
        return null;
        // return chooser.get();
    }


}