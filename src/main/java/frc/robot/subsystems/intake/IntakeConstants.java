package frc.robot.subsystems.intake;
import edu.wpi.first.math.system.plant.DCMotor;
import io.github.captainsoccer.basicmotor.BasicMotorConfig;
import io.github.captainsoccer.basicmotor.BasicMotor.IdleMode;
import io.github.captainsoccer.basicmotor.BasicMotorConfig.SlotConfig;
import io.github.captainsoccer.basicmotor.gains.ConstraintsGains.ConstraintType;
import io.github.captainsoccer.basicmotor.rev.BasicSparkConfig;

public class IntakeConstants {

    //The tolerance of the position of the intake
    public static final double POSITION_TOLERANCE = 0.05;
    

    //The positions of the intake (closed and opened)
    public static final double CLOSE_POSITION = 0;
    public static final double OPEN_POSITION = 0.35;

    public static final double INTAKE_VELOCITY = 4;

    public static final double PINION_DIAMETER = 0.033;

    public static final double TIME_FOR_INTAKE_TO_CLOSE = 4;
    public static final double TIME_FOR_INTAKE_TO_OPEN = 4;


    public static final BasicMotorConfig intakeMotorConfig = new BasicSparkConfig();
    static{
        intakeMotorConfig.motorConfig.name = "Intake motor";
        intakeMotorConfig.motorConfig.id = 22;
        intakeMotorConfig.motorConfig.inverted = true;
        intakeMotorConfig.motorConfig.idleMode = IdleMode.BRAKE; 
        intakeMotorConfig.motorConfig.gearRatio = 1 / (11.0/35);
        intakeMotorConfig.motorConfig.motorType = DCMotor.getNEO(1);

        intakeMotorConfig.slot0Config.pidConfig.kP = 0;
        intakeMotorConfig.slot0Config.pidConfig.kI = 0;
        intakeMotorConfig.slot0Config.pidConfig.kD = 0;
        intakeMotorConfig.slot0Config.pidConfig.tolerance = 0.1;

        intakeMotorConfig.simulationConfig.kA = 0.023275; // Not real value, taken from shooter for sim
        intakeMotorConfig.simulationConfig.kV = 0.31938; // Not real value, taken from shooter for sim

        var specifConfig = (BasicSparkConfig)intakeMotorConfig;

        specifConfig.currentLimitConfig.freeSpeedCurrentLimit = 60;
    }

    public static final BasicMotorConfig positionMotorConfig = new BasicSparkConfig();
    static{
        positionMotorConfig.motorConfig.name = "position motor";
        positionMotorConfig.motorConfig.id = 42;
        positionMotorConfig.motorConfig.inverted = false;
        positionMotorConfig.motorConfig.idleMode = IdleMode.BRAKE;
        positionMotorConfig.motorConfig.gearRatio = 5.4;
        positionMotorConfig.motorConfig.motorType = DCMotor.getNEO(1);
        positionMotorConfig.motorConfig.unitConversion = Math.PI * PINION_DIAMETER;

        SlotConfig posConfig = positionMotorConfig.slot0Config;
        posConfig.pidConfig.kP = 30;
        posConfig.pidConfig.kI = 0.5;
        posConfig.pidConfig.kD = 0;
        posConfig.pidConfig.iMaxAccum = 4;
        posConfig.pidConfig.iZone = 0.1;
        posConfig.pidConfig.tolerance = 0.003;

        SlotConfig posForSlowly = positionMotorConfig.slot1Config;
        posForSlowly.pidConfig.kP = 30;
        posForSlowly.pidConfig.kI = 0.5;
        posForSlowly.pidConfig.kD = 0;
        posForSlowly.pidConfig.iMaxAccum = 4;
        posForSlowly.pidConfig.iZone = 0.1;
        posForSlowly.pidConfig.tolerance = 0.003;
        posForSlowly.profileConfig.maximumMeasurementVelocity = 0.05;
        posForSlowly.profileConfig.maximumMeasurementAcceleration = 1;

        posConfig.profileConfig.maximumMeasurementVelocity = 10; 
        posConfig.profileConfig.maximumMeasurementAcceleration = 10;

//        posConfig.profileConfig.maximumMeasurementVelocity = 5;
//        posConfig.profileConfig.maximumMeasurementAcceleration = 10;

        positionMotorConfig.constraintsConfig.constraintType = ConstraintType.NONE;
        //positionMotorConfig.constraintsConfig.maxValue = 0.3;
       // positionMotorConfig.constraintsConfig.minValue = -0.01;



        positionMotorConfig.simulationConfig.kA = 0.023275; // Not real value, taken from shooter for sim
        positionMotorConfig.simulationConfig.kV = 0.31938; // Not real value, taken from shooter for sim

          var specifConfig = (BasicSparkConfig)positionMotorConfig;

        specifConfig.currentLimitConfig.freeSpeedCurrentLimit = 60;
    }
    
    public static class ResetConstants{
        public static final double INITIAL_DUTY_CYCLE_CHECK_FOR_CLOSE = -0.4;
        public static final double INITIAL_CURRENT_CHECK_FOR_CLOSE = 35;
        public static final double FINAL_POSITION_CHECK_FOR_CLOSE = 0.05;
        public static final double FINAL_DUTY_CYCLE_CHECK_FOR_CLOSE = -0.15;
        public static final double FINAL_CURRENT_CHECK_FOR_CLOSE = 30;
    }
}