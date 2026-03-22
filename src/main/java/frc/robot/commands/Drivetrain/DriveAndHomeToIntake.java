package frc.robot.commands.Drivetrain;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.utils.AllianceFlipUtil;

public class DriveAndHomeToIntake extends DriveAndHomeToSupplierCommand{

    private static final double RESET_ANGLE_ERROR = 50;
    private static final double MAX_ANGLE_ERROR = 90;
    private static final double MAX_ANGLE_ERROR_TIMEOUT = 4;
    
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

    private boolean ignoreMaxAngleError;

    private boolean shouldLookAtIntake(Rotation2d errorVectorAngle) {
        double error = Math.abs(errorVectorAngle.getDegrees());
        if(error <= RESET_ANGLE_ERROR) ignoreMaxAngleError = false;

        if(error <= MAX_ANGLE_ERROR || ignoreMaxAngleError){
            timer.stop();
            return true;
        }

        if(!timer.isRunning()) timer.restart();

        if(timer.hasElapsed(MAX_ANGLE_ERROR_TIMEOUT)){
            ignoreMaxAngleError = true;
            timer.stop();
            return true;
        }

        return false;
    }
}
