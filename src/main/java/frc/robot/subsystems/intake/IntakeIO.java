package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;

public interface IntakeIO {
    @AutoLog
    class IntakeInputs {
        Rotation2d Angle;
        double power ;
        double velocity;
    }

    boolean getBeamBrake();

    void stopMotor();

    void setPercentOutput(double percentOutput);

    Rotation2d getPos();

    void setActivationMotorPos(Rotation2d pos);

    void updateInputs(IntakeInputs inputs);





}
