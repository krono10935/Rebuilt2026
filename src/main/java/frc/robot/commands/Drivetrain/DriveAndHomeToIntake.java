package frc.robot.commands.Drivetrain;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.utils.AllianceFlipUtil;

public class DriveAndHomeToIntake extends DriveAndHomeToSupplierCommand{
    public DriveAndHomeToIntake(Drivetrain drivetrain, CommandXboxController controller) {
        super(drivetrain, controller, DriveAndHomeToIntake::getControllerVectorAngle);
    }

    private static ChassisSpeeds latestControllerInputs = new  ChassisSpeeds();

    @Override
    public ChassisSpeeds getControllerInputs() {
        latestControllerInputs = super.getControllerInputs();
        return latestControllerInputs;
    }

    private static Rotation2d getControllerVectorAngle() {
        return AllianceFlipUtil.apply(
                new Translation2d(latestControllerInputs.vxMetersPerSecond, latestControllerInputs.vyMetersPerSecond).getAngle());
    }

    @Override
    public double calculateThetaPID() {
        var driverVectorAngle = getControllerVectorAngle();

        if(!shouldLookAtIntake(driverVectorAngle.minus(drivetrain.getEstimatedPosition().getRotation()))){
            return 0;
        }

        return super.calculateThetaPID();
    }

    private final Timer timer = new Timer();

    private boolean flag;

    private boolean shouldLookAtIntake(Rotation2d errorVector) {
        double error = Math.abs(errorVector.getDegrees());
        if(error < 50) flag = false;

        if(error <= 90 || flag){
            timer.stop();
            return true;
        }

        if(!timer.isRunning()) timer.restart();

        if(timer.hasElapsed(4)){
            flag = true;
            timer.stop();
            return true;
        }

        return false;
    }
}
