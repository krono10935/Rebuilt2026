package frc.robot.subsystems.UpdateWigdets;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Vision.ObjectDetection.ObjectDetection;
import frc.utils.VirtualSubSystem;
import org.littletonrobotics.conduit.ConduitApi;

public class UpdateWidgets extends VirtualSubSystem{

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Are there Balls", ObjectDetection.getInstance().hasBalls());
        SmartDashboard.putNumber("Battery Voltage", ConduitApi.getInstance().getPDPVoltage());
        SmartDashboard.putBoolean("Intake Open", RobotContainer.getInstance().intake.isOpen());
        SmartDashboard.putBoolean("Shooter spun up", RobotContainer.getInstance().shooter.isKeepingVelocity());
    }

}


