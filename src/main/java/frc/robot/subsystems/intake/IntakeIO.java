package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
    @AutoLog
    class IntakeInputs {
        double temp;


    }

    public boolean getBeamBrake();

    public void stopMotor();

    public double getMotorPower();

    public double getPos();

    public void setPos(double pos);

    public void updateInputs(IntakeInputs inputs);





}
