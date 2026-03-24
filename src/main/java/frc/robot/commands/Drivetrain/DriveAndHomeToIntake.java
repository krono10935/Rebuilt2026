package frc.robot.commands.Drivetrain;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.utils.AllianceFlipUtil;

public class DriveAndHomeToIntake extends DriveAndHomeToSupplierCommand{

    private static final double MAX_ANGLE_ERROR = 90;
    
    public DriveAndHomeToIntake(Drivetrain drivetrain, CommandXboxController controller) {
        super(drivetrain, controller,  DriveAndHomeToIntake::getControllerVectorAngle);

        
    }

    private static ChassisSpeeds latestControllerInputs = new  ChassisSpeeds();

    @Override
    public ChassisSpeeds getControllerInputs() {
        latestControllerInputs = super.getControllerInputs();
        return latestControllerInputs;
    }

    private static Rotation2d getControllerVectorAngle() {
        return AllianceFlipUtil.apply(
                new Translation2d(latestControllerInputs.vxMetersPerSecond, latestControllerInputs.vyMetersPerSecond)
                .getAngle().plus(Rotation2d.kPi));
    }

    @Override
    public double calculateThetaPID() {
        var driverVectorAngle = getControllerVectorAngle();

        Logger.recordOutput("DriveAndHomeToIntake/error", driverVectorAngle);

        if(!shouldLookAtIntake(driverVectorAngle.minus(drivetrain.getEstimatedPosition().getRotation()))){
            return 0;
        }

        return super.calculateThetaPID();
    }


    private boolean shouldLookAtIntake(Rotation2d errorVectorAngle) {
        double error = Math.abs(errorVectorAngle.getDegrees());

        Logger.recordOutput("DriveAndHomeToIntake/error", error);

        return error <= MAX_ANGLE_ERROR;
    }
}
