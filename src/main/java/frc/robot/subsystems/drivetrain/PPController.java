//
// Source code recreated from a .class file by IntelliJ IDEA
// (powered by FernFlower decompiler)
//

package frc.robot.subsystems.drivetrain;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PathFollowingController;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class PPController implements PathFollowingController {
    private final PIDController xController;
    private final PIDController yController;
    private final PIDController rotationController;

    public PPController(PIDConstants translationConstants, PIDConstants rotationConstants, double period) {

        this.xController = new PIDController(translationConstants.kP, translationConstants.kI, translationConstants.kD, period);
        this.xController.setIZone(translationConstants.iZone);

        this.yController = new PIDController(translationConstants.kP, translationConstants.kI, translationConstants.kD, period);
        this.yController.setIZone(translationConstants.iZone);

        this.rotationController = new PIDController(rotationConstants.kP, rotationConstants.kI, rotationConstants.kD, period);
        this.rotationController.setIZone(rotationConstants.iZone);
        this.rotationController.enableContinuousInput(-Math.PI, Math.PI);
    }

    public PPController(PIDConstants translationConstants, PIDConstants rotationConstants) {
        this(translationConstants, rotationConstants, 0.02);
    }


    public void reset(Pose2d currentPose, ChassisSpeeds currentSpeeds) {
        this.xController.reset();
        this.yController.reset();
        this.rotationController.reset();
    }

    public ChassisSpeeds calculateRobotRelativeSpeeds(Pose2d currentPose, PathPlannerTrajectoryState targetState) {
        double xFF = targetState.fieldSpeeds.vxMetersPerSecond;
        double yFF = targetState.fieldSpeeds.vyMetersPerSecond;

        double xFeedback = this.xController.calculate(currentPose.getX(), targetState.pose.getX());
        double yFeedback = this.yController.calculate(currentPose.getY(), targetState.pose.getY());

        var targetPose = targetState.pose;
        double rotationFF = targetState.fieldSpeeds.omegaRadiansPerSecond;

        double rotationFeedback = this.rotationController.calculate(currentPose.getRotation().getRadians(), targetPose.getRotation().getRadians());

        return ChassisSpeeds.fromFieldRelativeSpeeds(xFF + xFeedback, yFF + yFeedback, rotationFF + rotationFeedback, currentPose.getRotation());
    }

    public boolean isHolonomic() {
        return true;
    }
}
