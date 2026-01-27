// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.Shooter.KeepVelocity;
import frc.robot.commands.Shooter.SpinUp;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Shooter.ShooterSysID;
import frc.robot.subsystems.drivetrain.Drivetrain;

import org.littletonrobotics.conduit.ConduitApi;

import edu.wpi.first.wpilibj2.command.*;


public class RobotContainer
{

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

    private RobotContainer()
    {

        shooter = new Shooter();

        xboxController = new CommandXboxController(0);

        drivetrain = new Drivetrain(ConduitApi.getInstance()::getPDPVoltage, Constants.CHASSIS_TYPE.constants);

        configureBindings();
        // chooser = new LoggedDashboardChooser<>("chooser", AutoBuilder.buildAutoChooser());
    }

    private void configureBindings() {

        xboxController.a()
        .whileTrue(new SpinUp(shooter).andThen(new KeepVelocity(shooter)))
        .onFalse(new InstantCommand(()-> shooter.stopFlyWheel()).ignoringDisable(true));
    }
    
    
    public Command getAutonomousCommand()
    {
        return null;
        // return chooser.get();
    }


}