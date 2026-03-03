package frc.robot.subsystems.UpdateWigdets;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Vision.ObjectDetection.ObjectDetection;
import frc.utils.VirtualSubSystem;
import org.littletonrobotics.conduit.ConduitApi;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class UpdateWidgets extends VirtualSubSystem{

    private LoggedDashboardChooser<Boolean> whoWonTheAuto;

    public UpdateWidgets(){
        whoWonTheAuto = new LoggedDashboardChooser<>("DidoeStart");
    }


    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Are there Balls", ObjectDetection.getInstance().hasBalls());
        SmartDashboard.putNumber("Battery Voltage", ConduitApi.getInstance().getPDPVoltage());
        SmartDashboard.putBoolean("Intake Open", RobotContainer.getInstance().intake.isOpen());
        SmartDashboard.putBoolean("Shooter spun up", RobotContainer.getInstance().shooter.isKeepingVelocity());
    }

}


