package frc.robot.commands;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.drivetrain.Drivetrain;

/**
 * Swerve SYSID command factory
 */
public class SwerveSysID {
    /**
     * SysID routine for Drive
     */
   private SysIdRoutine routineDrive;
    /**
     * SysID routine for Steer
     */
   private SysIdRoutine routineSteer;
    /**
     * SysID routine for Spin
     */
   private SysIdRoutine routineSpin;
    /**
     * Controller to steer the robot in Drive routine
     */
   private CommandXboxController controller;
    /**
     * Robot's drivetrain to apply the sysID routines to
     */
   private Drivetrain drivetrain;


    /**
     * Voltage to use in dynamic mode for SYSID
     */
   public static final double VOLT = 4;

    /**
     * How many volts/second to add per second of the SYSID routine
     */
   public static final double VOLT_RAMP_RATE = 4;

    /**
     * How many seconds to perform the test for the sysID routine
     */
   public static final double TIMEOUT = 10;

    /**
     * Builds the Swerve SYSID check
     * @param drivetrain Which drivetrain to apply the SYSID routine to
     * @param controller Controller to steer the drivetrain during Drive mode
     */

     
   public SwerveSysID(Drivetrain drivetrain, CommandXboxController controller) {
      this.controller = controller;
      this.drivetrain = drivetrain;    
                

            routineSteer = new SysIdRoutine(
              new SysIdRoutine.Config(Units.Volts.per(Units.Second).of(VOLT_RAMP_RATE),
                      Volts.of(VOLT),
                      Second.of(TIMEOUT),
                      (state) -> Logger.recordOutput("SysIdTestState", state.toString())
              ),

              new SysIdRoutine.Mechanism(
                      (volt) -> drivetrain.setSteerVoltage(volt.baseUnitMagnitude()),
                      null,
                      drivetrain
              )
      );

      routineDrive = new SysIdRoutine(
              new SysIdRoutine.Config(Units.Volts.per(Units.Second).of(VOLT_RAMP_RATE),
                      Volts.of(VOLT),
                      Second.of(TIMEOUT),
                      (state) -> Logger.recordOutput("SysIdTestState", state.toString())
              ),

              new SysIdRoutine.Mechanism(
                      (volt) -> this.driveWithController(volt.baseUnitMagnitude()),
                      null,
                      drivetrain
              )
      );

      routineSpin = new SysIdRoutine(
              new SysIdRoutine.Config(Units.Volts.per(Units.Second).of(VOLT_RAMP_RATE),
                      Volts.of(VOLT),
                      Second.of(TIMEOUT),
                      (state) -> Logger.recordOutput("SysIdTestState", state.toString())
              ),
              new SysIdRoutine.Mechanism(
                      (volt) -> this.spin(volt.baseUnitMagnitude()),
                      null,
                      drivetrain
              )
      );
   }

    /**
     * Translate an angle relative to the field to be relative to the robot (30 degrees field turns into 30 degrees robot)
     * @param rotation Original field relative angle
     * @param drivetrain The robot to rotate the angle based on it's position in the field
     * @return The angle on the field translated to the robot relative angle
     */
   public static Rotation2d fieldRelativeToRobotRelative(Rotation2d rotation, Drivetrain drivetrain) {
       return rotation.rotateBy(drivetrain.getGyroAngle().unaryMinus());
   }
    /**
     * Drive with a certain amount of power
     * @param voltage Power to apply to the drivetrain
     */
   public void driveWithController(double voltage) {

      Rotation2d fieldRelativeAngle = new Translation2d(-controller.getLeftY(), -controller.getLeftX()).getAngle();
      Rotation2d robotRelativeAngle = fieldRelativeToRobotRelative(fieldRelativeAngle, drivetrain);
      Rotation2d[] angleArray = {robotRelativeAngle, robotRelativeAngle, robotRelativeAngle, robotRelativeAngle};

      drivetrain.setDriveVoltageAndSteerAngle(voltage, angleArray);
   }

    /**
     * Angles to put the wheels at for spin
     */
   private final Rotation2d[] spinAngleArray = {
           Rotation2d.fromDegrees(135),
           Rotation2d.fromDegrees(135-90),
           Rotation2d.fromDegrees(135+90),
           Rotation2d.fromDegrees(135+180)
   };

    /**
     * Spin the robot at a certain power
     * @param voltage How much power to apply in the spin
     */
   public void spin(double voltage) {
      drivetrain.setDriveVoltageAndSteerAngle(voltage, spinAngleArray);
   }


    /**
     * Factory for the Quasistatic Drive command
     * @param direction Apply the test forward or backward
     * @return A command to apply the sysID test
     */
   public Command sysIdQuasistaticDrive(SysIdRoutine.Direction direction) {
      return routineDrive.quasistatic(direction);
   }

    /**
     * Factory for the Dynamic Drive command
     * @param direction Apply the test forward or backward
     * @return A command to apply the sysID test
     */
   public Command sysIdDynamicDrive(SysIdRoutine.Direction direction) {
      return routineDrive.dynamic(direction);
   }

    /**
     * Factory for the Quasistatic Steer command
     * @param direction Apply the test forward or backward
     * @return A command to apply the sysID test
     */
   public Command sysIdQuasistaticSteer(SysIdRoutine.Direction direction) {
      return routineSteer.quasistatic(direction);
   }

    /**
     * Factory for the Dynamic Steer command
     * @param direction Apply the test forward or backward
     * @return A command to apply the sysID test
     */
   public Command sysIdDynamicSteer(SysIdRoutine.Direction direction) {
      return routineSteer.dynamic(direction);
   }

    /**
     * Factory for the Quasistatic Spin command
     * @param direction Apply the test forward or backward
     * @return A command to apply the sysID test
     */
   public Command sysIdQuasistaticSpin(SysIdRoutine.Direction direction) {
      return routineSpin.quasistatic(direction);
   }

    /**
     * Factory for the Dynamic Spin command
     * @param direction Apply the test forward or backward
     * @return A command to apply the sysID test
     */
   public Command sysIdDynamicSpin(SysIdRoutine.Direction direction) {
      return routineSpin.dynamic(direction);
   }
}
