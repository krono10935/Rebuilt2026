// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drivetrain;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.drivetrain.Drivetrain;



public class DriveRobotRelative extends DriveCommand {

    public DriveRobotRelative(Drivetrain drivetrain, CommandXboxController controller) {
        super(drivetrain, controller);
    }


    @Override
    public void execute() {
        ChassisSpeeds speeds = getControllerInputs();

        drivetrain.drive(new ChassisSpeeds(speeds.vxMetersPerSecond,speeds.vyMetersPerSecond,speeds.omegaRadiansPerSecond));
    }
}
