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
    private final DoubleSupplier currentOutputSupplierPosition;
    private final DoubleSupplier currentOutputSupplierIntake;
    public IntakeIOSpark() {

        intakeMotor = new BasicSparkFlex(IntakeConstants.intakeMotorConfig);

        positionMotor = new BasicSparkMAX(IntakeConstants.positionMotorConfig);
        
        //position
        BasicSparkMAX rootPositionMotor = ((BasicSparkMAX)positionMotor); 
        SparkBase positionMotorSpark = rootPositionMotor.getMotor(); 

        currentOutputSupplierPosition = positionMotorSpark::getOutputCurrent; 

        rootPositionMotor.getSparkConfig().signals.outputCurrentPeriodMs(10); 

        positionMotorSpark.configure(rootPositionMotor.getSparkConfig(),ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters); 

        //intake
        BasicSparkFlex rootIntakeMotor = ((BasicSparkFlex)intakeMotor);
        SparkBase intakeMotorSpark = rootIntakeMotor.getMotor();

        currentOutputSupplierIntake = intakeMotorSpark::getOutputCurrent;

        rootIntakeMotor.getSparkConfig().signals.outputCurrentPeriodMs(10);

        intakeMotorSpark.configure(rootIntakeMotor.getSparkConfig(),ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    private boolean intakeMotorAtSetPoint() {
        return intakeMotor.atGoal();
    }


    @Override
    public void stopIntakeMotor() {
        intakeMotor.stop();
    }

    @Override
    public void stopPositiongMotor() {
        positionMotor.stop();
    }

    private boolean positionMotorAtSetPoint() {
        return positionMotor.atGoal();
    }

    @Override
    public void setPositionMotor(double pos) {
        positionMotor.setControl(pos, ControlMode.PROFILED_POSITION, 0);
    }

    private double getSpeedPositionMotor() {
        return positionMotor.getVelocity();
    }

    @Override
    public void setIntakeMotorPercent(double dutyCycle){
        intakeMotor.setPercentOutput(dutyCycle);
    }

    @Override
    public void updateInputs(IntakeInputs inputs) {
        inputs.intakePositionMeters = positionMotor.getPosition();
        inputs.intakeMotorVelocityMPS = intakeMotor.getVelocity();
        inputs.positionMotorCurrentAmps = currentOutputSupplierPosition.getAsDouble();
        inputs.intakeMotorCurrentAmps = currentOutputSupplierIntake.getAsDouble();
        inputs.isIntakeMotorAtSetPoint = intakeMotorAtSetPoint();
        inputs.isPositionMotorAtSetPoint = positionMotorAtSetPoint();
        inputs.positionMotorVelocityMPS = getSpeedPositionMotor();
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
