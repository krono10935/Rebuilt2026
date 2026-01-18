package frc.robot.commands.Shooter;

import java.util.function.Function;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter.Shooter;

public class ShooterCommandFactory {

    /**
     * 
     * @param shooter shooter subsystem to activate the command on
     * @param shouldShoot function to indicate whether or not the robot should shoot based on it's posed2d
     * @param poseToHoodAngle desired angle based on the pose of the robot
     * @param robotPoseSupplier supplier of the pose of the robot
     * @return a command to shoot and aim the shooter based on its location
     */
    public static Command AutoShoot(Shooter shooter,
     Function<Pose2d,Boolean> shouldShoot, Function<Pose2d,Rotation2d> poseToHoodAngle, 
     Supplier<Pose2d> robotPoseSupplier){

        return new ShootCommand(shooter, robotPoseSupplier, shouldShoot).alongWith(new HomeHoodCommand(shooter, robotPoseSupplier, poseToHoodAngle));

    }

}