package frc.robot.subsystems.intake;

import java.util.function.DoubleSupplier;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase;

import io.github.captainsoccer.basicmotor.BasicMotor;
import io.github.captainsoccer.basicmotor.controllers.Controller.ControlMode;
import io.github.captainsoccer.basicmotor.rev.BasicSparkFlex;
import io.github.captainsoccer.basicmotor.rev.BasicSparkMAX;

public class IntakeIOSpark implements IntakeIO {
    private final BasicMotor intakeMotor;
    private final BasicMotor positionMotor;
    private final DoubleSupplier currentOutputSupplier;
    public IntakeIOSpark() {

        intakeMotor = new BasicSparkFlex(IntakeConstants.intakeMotorConfig);

        positionMotor = new BasicSparkMAX(IntakeConstants.positionMotorConfig);


        BasicSparkMAX motor = ((BasicSparkMAX)positionMotor);

        SparkBase positionMotorSpark = motor.getMotor();

        currentOutputSupplier = positionMotorSpark::getOutputCurrent;
 
        motor.getSparkConfig().signals.outputCurrentPeriodMs(10);

        positionMotorSpark.configure(motor.getSparkConfig(),ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    @Override
    public boolean intakeMotorAtSetPoint() {
        return intakeMotor.atGoal();
    }

    @Override
    public void setIntake90PercentSpeed(double velocity) {
        intakeMotor.setPercentOutput(0.9);
        // intakeMotor.setControl(velocity, ControlMode.VELOCITY);
    }

    @Override
    public void stopIntakeMotor() {
        intakeMotor.stop();
    }

    @Override
    public void stopPositiongMotor() {
        positionMotor.stop();
    }

    @Override
    public boolean positionMotorAtSetPoint() {
        return positionMotor.atGoal();
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
    public double getSpeedPositionMotor() {
        return positionMotor.getVelocity();
    }

    @Override
    public void setIntakeMotorPercent(double dutyCycle){
        intakeMotor.setPercentOutput(dutyCycle);
    }

    @Override
    public void updateInputs(IntakeInputs inputs) {
        inputs.position = positionMotor.getPosition();
        inputs.velocity = intakeMotor.getVelocity();
        inputs.positionMotorCurrent = currentOutputSupplier.getAsDouble();
    }

    @Override
    public boolean isInPositionControl() {
        return positionMotor.getController().getControlMode().isPositionControl();
    }

    @Override
    public boolean isInVelocityControl() {
        return positionMotor.getController().getControlMode().isVelocityControl();
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
        positionMotor.setControl(posMeters, ControlMode.PROFILED_POSITION, 1);
    }
}
