package frc.robot.subsystems.intake;

import java.util.function.DoubleSupplier;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import io.github.captainsoccer.basicmotor.BasicMotor;
import io.github.captainsoccer.basicmotor.controllers.Controller.ControlMode;
import io.github.captainsoccer.basicmotor.rev.BasicSparkMAX;

public class IntakeIOSpark implements IntakeIO {
    private final BasicMotor intakeMotor;
    private final BasicMotor positionMotor;
    private final DoubleSupplier currentOutputSupplierPosition;
    private final DoubleSupplier currentOutputSupplierIntake;
    public IntakeIOSpark() {

        intakeMotor = new BasicSparkMAX(IntakeConstants.intakeMotorConfig);

        positionMotor = new BasicSparkMAX(IntakeConstants.positionMotorConfig);

        SmartDashboard.putData(positionMotor.getController());
        
        //position
        BasicSparkMAX rootPositionMotor = ((BasicSparkMAX)positionMotor); 
        SparkBase positionMotorSpark = rootPositionMotor.getMotor(); 

        currentOutputSupplierPosition = positionMotorSpark::getOutputCurrent; 

        rootPositionMotor.getSparkConfig().signals.outputCurrentPeriodMs(10); 

        positionMotorSpark.configure(rootPositionMotor.getSparkConfig(),ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters); 

        //intake
        BasicSparkMAX rootIntakeMotor = ((BasicSparkMAX)intakeMotor);
        SparkBase intakeMotorSpark = rootIntakeMotor.getMotor();

        currentOutputSupplierIntake = intakeMotorSpark::getOutputCurrent;

        rootIntakeMotor.getSparkConfig().signals.outputCurrentPeriodMs(10);

        intakeMotorSpark.configure(rootIntakeMotor.getSparkConfig(),ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
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
        inputs.positionMotorCurrent = currentOutputSupplierPosition.getAsDouble();
        inputs.intakeMotorCurrent = currentOutputSupplierIntake.getAsDouble();
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
