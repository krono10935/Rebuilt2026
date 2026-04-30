package frc.robot.subsystems.UpdateWigdets;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.HubTiming;
import frc.robot.FieldConstants.Hub;
import frc.robot.subsystems.Shooter.ShotCalculator;
import frc.robot.subsystems.Vision.ObjectDetection.ObjectDetection;
import frc.robot.subsystems.intake.IntakeConstants.IntakeMode;
import frc.utils.AllianceFlipUtil;
import org.littletonrobotics.conduit.ConduitApi;
import org.littletonrobotics.junction.Logger;

import java.util.Objects;

public class UpdateWidgets extends SubsystemBase {

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Are there Balls", ObjectDetection.getInstance().hasBalls());
        SmartDashboard.putNumber("Battery Voltage", ConduitApi.getInstance().getPDPVoltage());
        SmartDashboard.putBoolean("Intake Open", RobotContainer.getInstance().intake.isFullyOpen());
        SmartDashboard.putBoolean("Intake moving", RobotContainer.getInstance().intake.isMoving());
        SmartDashboard.putBoolean("Shooter spun up", RobotContainer.getInstance().shooter.isKeepingVelocity());


        double time = DriverStation.getMatchTime();
        SmartDashboard.putNumber("MatchTime", time);
        
        var drivetrain = RobotContainer.getInstance().drivetrain;

        var chassisSpeeds = drivetrain.getChassisSpeeds();

        SmartDashboard.putNumber("Velocity", Math.hypot(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond));

        double distanceToHub = AllianceFlipUtil.apply(Hub.innerCenterPoint.toTranslation2d())
            .getDistance(drivetrain.getEstimatedPosition().getTranslation());

        var params = ShotCalculator.getInstance().getParameters(drivetrain.getEstimatedPosition(), chassisSpeeds);

        SmartDashboard.putBoolean("IsHubActive", HubTiming.isActive(time));

        SmartDashboard.putNumber("timeToNextShift", HubTiming.timeToNextShift(time));

        var driveTrainCommand = drivetrain.getCurrentCommand();
        SmartDashboard.putBoolean("NormalDrive",
                driveTrainCommand != null && Objects.equals(driveTrainCommand.getName(), "DriveCommand"));

        SmartDashboard.putBoolean("overrideShooting", RobotContainer.getInstance().overrideShooting);

        SmartDashboard.putNumber("RobotAngleOffset", params.robotAngleOffset().getDegrees());
        SmartDashboard.putNumber("HoodAngleOffset", params.hoodAngleOffset().getDegrees());
        SmartDashboard.putNumber("FlyWheelSpeedOffset", params.flyWheelOffset());
        SmartDashboard.putNumber("distanceToHub", distanceToHub);

        for (IntakeMode mode : IntakeMode.values()){
            boolean isOn = mode == RobotContainer.getInstance().currentIntakeMode;
            Logger.recordOutput("IntakeMode" , mode.name());
            SmartDashboard.putBoolean(mode.name(), isOn);
        }
    }

}


