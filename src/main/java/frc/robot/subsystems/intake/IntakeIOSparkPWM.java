package frc.robot.subsystems.intake;

import java.util.function.DoubleSupplier;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase;

import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import io.github.captainsoccer.basicmotor.BasicMotor;
import io.github.captainsoccer.basicmotor.controllers.Controller.ControlMode;
import io.github.captainsoccer.basicmotor.rev.BasicSparkMAX;

public class IntakeIOSparkPWM implements IntakeIO {
    private final PWMSparkMax intakeMotor;
    private final BasicMotor positionMotor;
    private double dutyCycle;
    private final DoubleSupplier currentOutputSupplierPosition;
    public IntakeIOSparkPWM() {

        intakeMotor = new PWMSparkMax(IntakeConstants.INTAKE_MOTOR_CHANNEL);

        positionMotor = new BasicSparkMAX(IntakeConstants.positionMotorConfig);
        
        //position
        BasicSparkMAX rootPositionMotor = ((BasicSparkMAX)positionMotor); 
        SparkBase positionMotorSpark = rootPositionMotor.getMotor(); 

        currentOutputSupplierPosition = positionMotorSpark::getOutputCurrent; 

        dutyCycle = 0;

        rootPositionMotor.getSparkConfig().signals.outputCurrentPeriodMs(10); 

        positionMotorSpark.configure(rootPositionMotor.getSparkConfig(),ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters); 

    }

    private boolean intakeMotorAtSetPoint() {
        return true;
    }


    @Override
    public void stopIntakeRoller() {
        intakeMotor.set(0.0);
        this.dutyCycle = 0;
    }

    @Override
    public void stopPositionMotor() {
        positionMotor.stop();
    }

    private boolean positionMotorAtSetPoint() {
        return positionMotor.atGoal();
    }

    @Override
    public void moveToPosition(double pos) {
        positionMotor.setControl(pos, ControlMode.PROFILED_POSITION, 0);
    }

    private double getSpeedPositionMotor() {
        return positionMotor.getVelocity();
    }

    @Override
    public void setRollerDutyCycle(double dutyCycle){
        intakeMotor.set(dutyCycle);
    }

    @Override
    public void updateInputs(IntakeInputs inputs) {
        inputs.intakePositionMeters = positionMotor.getPosition();
        inputs.rollerMotorVelocityMPS = dutyCycle;
        inputs.positionMotorCurrentAmps = currentOutputSupplierPosition.getAsDouble();
        inputs.isRollerMotorAtSetPoint = intakeMotorAtSetPoint();
        inputs.isPositionMotorAtSetPoint = positionMotorAtSetPoint();
        inputs.positionMotorVelocityMPS = getSpeedPositionMotor();
    }

    @Override
    public void resetOpeningMotorEncoder(double posMeters) {
        positionMotor.resetEncoder(posMeters);
    }

    @Override
    public void setPositionMotorDutyCycle(double dutyCycle) {
        positionMotor.setPercentOutput(dutyCycle);
    }

    @Override
    public void moveToPositionSlowly(double posMeters){
        positionMotor.setControl(posMeters, ControlMode.PROFILED_POSITION, 1);
    }
}
