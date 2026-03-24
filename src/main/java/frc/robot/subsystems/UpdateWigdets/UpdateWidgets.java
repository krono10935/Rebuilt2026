package frc.robot.subsystems.UpdateWigdets;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotContainer;
import frc.robot.Constants.HubTiming;
import frc.robot.FieldConstants.Hub;
import frc.robot.subsystems.Shooter.ShotCalculator;
import frc.robot.subsystems.Vision.ObjectDetection.ObjectDetection;
import frc.utils.AllianceFlipUtil;
import frc.utils.VirtualSubSystem;
import org.littletonrobotics.conduit.ConduitApi;

public class UpdateWidgets extends VirtualSubSystem{

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Are there Balls", ObjectDetection.getInstance().hasBalls());
        SmartDashboard.putNumber("Battery Voltage", ConduitApi.getInstance().getPDPVoltage());
        SmartDashboard.putBoolean("Intake Open", RobotContainer.getInstance().intake.isOpen());
        SmartDashboard.putBoolean("Intake moving", RobotContainer.getInstance().intake.isMoving());
        SmartDashboard.putBoolean("Shooter spun up", RobotContainer.getInstance().shooter.isKeepingVelocity());

        SmartDashboard.putNumber("MatchTime", DriverStation.getMatchTime());
        
        var drivetrain = RobotContainer.getInstance().drivetrain;

        var chassisSpeeds = drivetrain.getChassisSpeeds();

        SmartDashboard.putNumber("Velocity", Math.hypot(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond));

        double distanceToHub = AllianceFlipUtil.apply(Hub.innerCenterPoint.toTranslation2d())
            .getDistance(drivetrain.getEstimatedPosition().getTranslation());

        var params = ShotCalculator.getInstance().getParameters(drivetrain.getEstimatedPosition(), chassisSpeeds);
        
        SmartDashboard.putNumber("RobotAngleOffset", params.robotAngleOffset().getDegrees());
        SmartDashboard.putNumber("HoodAngleOffset", params.hoodAngleOffset().getDegrees());
        SmartDashboard.putNumber("FlyWheelSpeedOffset", params.flyWheelOffset());
        SmartDashboard.putNumber("distanceToHub", distanceToHub);


        SmartDashboard.putBoolean("IsHubActive", HubTiming.isActive(DriverStation.getMatchTime()));

    }

}


