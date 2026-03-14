package frc.robot.subsystems.intake;

import java.util.function.DoubleSupplier;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import io.github.captainsoccer.basicmotor.BasicMotor;
import io.github.captainsoccer.basicmotor.controllers.Controller.ControlMode;
import io.github.captainsoccer.basicmotor.rev.BasicSparkMAX;

public class IntakeIOSpark implements IntakeIO {
    private final BasicMotor intakeMotor;
    private final BasicMotor positionMotor;
    private final DoubleSupplier currentOutputSupplier;
    public IntakeIOSpark() {

        intakeMotor = new BasicSparkMAX(IntakeConstants.intakeMotorConfig);

        positionMotor = new BasicSparkMAX(IntakeConstants.positionMotorConfig);

        SmartDashboard.putData(positionMotor.getController());

        var motor = ((BasicSparkMAX)positionMotor);

        var positionMotorSpark = motor.getMotor();

        currentOutputSupplier = positionMotorSpark::getOutputCurrent;
 
        motor.getSparkConfig().signals.outputCurrentPeriodMs(10);

        positionMotorSpark.configure(motor.getSparkConfig(),ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    @Override
    public boolean intakeMotorAtSetPoint() {
        return intakeMotor.atSetpoint();
    }

    @Override
    public void setIntakeMotorVelocity(double velocity) {
        intakeMotor.setPercentOutput(0.5);
        // intakeMotor.setControl(velocity, ControlMode.VELOCITY);
    }

    @Override
    public void stopIntakeMotor() {
        intakeMotor.stop();
    }

    @Override
    public void stopIntakeOpeningMotor() {
        positionMotor.stop();
    }

    @Override
    public void setPositionMotorPercentOutput(double percent){
        positionMotor.setPercentOutput(percent);
    }

    @Override
    public boolean positionMotorAtSetPoint() {
        return positionMotor.atSetpoint();
    }

    @Override
    public void setPositionMotorVelocity(Rotation2d velocity){
        positionMotor.setControl(velocity.getRotations(), ControlMode.VELOCITY, 1);
    }

    @Override
    public double getIntakePosition() {
        return positionMotor.getPosition();
    }

    @Override
    public void setPositionMotor(double pos) {
        positionMotor.setControl(pos, ControlMode.PROFILED_POSITION, 0);
    }

    @Override
    public Rotation2d getSpeedPositionMotor() {
        return Rotation2d.fromRotations(positionMotor.getVelocity());
    }

    @Override
    public void setIntakeMotorPercent(double dutyCycle){
        intakeMotor.setPercentOutput(dutyCycle);
    }


    @Override
    public void updateInputs(IntakeInputs inputs) {
        inputs.position = positionMotor.getPosition();
        inputs.velocity = intakeMotor.getVelocity(); 
    }

    @Override
    public boolean isInPositionControl() {
        return positionMotor.getController().getControlMode().isPositionControl();
    }

    @Override
    public void resetPositionMotor(double posMeters) {
        positionMotor.resetEncoder(posMeters);
    }

    @Override
    public void setPositionMotorPercent(double dutyCycle) {
        positionMotor.setPercentOutput(dutyCycle);
    }

    @Override
    public void setPositionMotorSlowly(double posMeters){
        positionMotor.setControl(posMeters,ControlMode.PROFILED_POSITION, 1);
    }

    @Override
    public double getIntakePositionMotorVelocity() {
        return positionMotor.getVelocity();
    }

    @Override
    public double getPositionMotorCurrent() {
        return currentOutputSupplier.getAsDouble();
    }
}
