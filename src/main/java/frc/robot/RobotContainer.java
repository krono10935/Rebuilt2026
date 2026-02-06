// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.DriveCommand;
import frc.robot.subsystems.Vision.Vision;
import frc.robot.subsystems.Vision.Vision.VisionConsumer;
import frc.robot.subsystems.drivetrain.Drivetrain;

import java.util.function.Supplier;

import org.littletonrobotics.conduit.ConduitApi;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.DriveToPose;
import com.pathplanner.lib.path.DriveToPoseConstants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.*;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;


public class RobotContainer
{

    public static Vision vision;
    private static RobotContainer instance;

    private LoggedNetworkNumber poseX;
    private LoggedNetworkNumber poseY;

    private LoggedNetworkNumber poseYaw;




    //public final Drivetrain drivetrain;

    //private final LoggedDashboardChooser<Command> chooser;


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

        poseYaw = new LoggedNetworkNumber("PoseYaw", 0);

        lastPoseSupplier = () -> Pose2d.kZero;

        vision = new Vision(VisionConsumer.NO_OP, lastPoseSupplier);

        //drivetrain = new Drivetrain(ConduitApi.getInstance()::getPDPVoltage, Constants.CHASSIS_TYPE.constants);

        configureBindings();
        //chooser = new LoggedDashboardChooser<>("chooser", AutoBuilder.buildAutoChooser());
    }

    private void configureBindings() {

    }
    
    
    public Command getAutonomousCommand()
    {
        return null;//chooser.get();
    }


}