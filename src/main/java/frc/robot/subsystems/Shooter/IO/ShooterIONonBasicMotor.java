// /*
//  * 
//  * 
//  * 
//  * 
//  * DO NOT TOUCH THIS BLACK MAGIC
//  * 
//  * 
//  * 
//  * 
//  */








// package frc.robot.subsystems.Shooter.IO;

// import com.revrobotics.PersistMode;
// import com.revrobotics.ResetMode;
// import com.revrobotics.spark.SparkFlex;
// import com.revrobotics.spark.SparkLowLevel.MotorType;
// import com.revrobotics.spark.config.SparkFlexConfig;

// import edu.wpi.first.math.controller.LinearQuadraticRegulator;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.system.plant.DCMotor;
// import frc.robot.subsystems.Shooter.ShooterConstants;
// import frc.robot.subsystems.Shooter.ShooterIO;
// import io.github.captainsoccer.basicmotor.BasicMotor;
// import io.github.captainsoccer.basicmotor.BasicMotorConfig;
// import io.github.captainsoccer.basicmotor.controllers.Controller.ControlMode;
// import io.github.captainsoccer.basicmotor.rev.BasicSparkFlex;

// public class ShooterIONonBasicMotor implements ShooterIO {
//     private final SparkFlex leadShootingMotor;
//     private final SparkFlex followShootingMotor;

//     private boolean hasDoneCycle;

//     private final BasicMotorConfig leadConfig;

//     private ShooterInputs lastInputs;

//     private boolean isKickerActive;
//     private double targetVelocity;
//     public ShooterIONonBasicMotor(){
//         hasDoneCycle = false;

//         leadConfig = ShooterConstants.getLeadShootingMotorConfig();

//         leadShootingMotor = new SparkFlex(leadConfig.motorConfig.id, MotorType.kBrushless);
//         followShootingMotor = new SparkFlex(ShooterConstants.getLeadShootingMotorConfig().motorConfig.id, MotorType.kBrushless);
        
//         SparkFlexConfig leadFlexConfig = new SparkFlexConfig();
//         leadFlexConfig.closedLoop.feedForward.kA(leadConfig.simulationConfig.kA);
//         leadFlexConfig.closedLoop.feedForward.kV(leadConfig.simulationConfig.kV);
//         leadFlexConfig.closedLoop.feedForward.kS(leadConfig.slot0Config.feedForwardConfig.frictionFeedForward);

//         leadShootingMotor.configure(leadFlexConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

//         SparkFlexConfig followFlexConfig = 
//         (SparkFlexConfig)(new SparkFlexConfig().follow(leadConfig.motorConfig.id, ShooterConstants.FLYWHEEL_MOTORS_OPPOSITE));
        

//         followShootingMotor.configure(followFlexConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

//         isKickerActive = false;
//     }

//     @Override
//     public void spinUp(double speedMPS){
//         targetVelocity = speedMPS;
//         leadShootingMotor.getClosedLoopController().setSetpoint(speedMPS, null)
//     }

//     @Override
//     public void keepVelocity(){
//         double kA = 0;
//         double accel = lastInputs.acceleration;
//         if(accel < 0){
//             kA = -accel * leadConfig.simulationConfig.kA;
//         }
//         leadShootingMotor.setControl(targetVelocity , ControlMode.VELOCITY, kA, 0);
//     }

//     public void resetHasDoneCycle(){
//         hasDoneCycle = false;
//     }

//     private void internalUpdate(){
//         lastInputs.hoodAngle = Rotation2d.kZero;
//         lastInputs.isKickerActive = isKickerActive;
//         if (!hasDoneCycle){
//             double newSpeed = leadShootingMotor.get() * leadConfig.motorConfig.motorType.freeSpeedRadPerSec;
//             lastInputs.acceleration = (newSpeed - lastInputs.shooterSpeed) / 0.02;

//             lastInputs.shooterSpeed = newSpeed;
//             hasDoneCycle = true;
//         }
//     }

//     public void update(ShooterInputs inputs){
//         lastInputs = 
//     }

    
// }
