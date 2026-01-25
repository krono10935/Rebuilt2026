package frc.robot.subsystems.Shooter;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

/**
 * Swerve SYSID command factory
 */
public class ShooterSysID {
    /**
     * SysID routine for Drive
     */
   private SysIdRoutine flyWheelRoutine;

   /**
     * Robot's shooter to apply the sysID routine to
     */
   private Shooter shooter;

    /**
     * Voltage to use in dynamic mode for SYSID
     */
   public static final double VOLT = 2;

    /**
     * How many volts/second to add per second of the SYSID routine
     */
   public static final double VOLT_RAMP_RATE = 0.5;

    /**
     * How many seconds to perform the test for the sysID routine
     */
   public static final double TIMEOUT = 10;

  /**
   * 
   * @param shooter the shooter subsystem to apply the sys id to
   */
   public ShooterSysID(Shooter shooter) {
      this.shooter = shooter; 
                

      flyWheelRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(Units.Volts.per(Units.Second).of(VOLT_RAMP_RATE),
                    Volts.of(VOLT),
                    Second.of(TIMEOUT),
                    (state) -> {
                        Logger.recordOutput("SysIdTestState", state.toString());
                        shooter.logSysID();
                    }
            ),

            new SysIdRoutine.Mechanism(
                    (volt) -> shooter.setVoltage(volt.baseUnitMagnitude()),
                    null,
                    shooter
            )
      );
   }

   /**
    * 
    * @param direction direction in which to apply the Quastatic sysid routine of the flywheel
    * @return command the apply the Quastatic sysid test based on the direction specified
    */
   public Command sysIdQuasistaticFlywheel(SysIdRoutine.Direction direction) {
      return flyWheelRoutine.quasistatic(direction);
   }


    /**
    * 
    * @param direction direction in which to apply the Dynamic sysid routine of the flywheel
    * @return command the apply the Dynamic sysid test based on the direction specified
    */
   public Command sysIdDynamicFlywheel(SysIdRoutine.Direction direction) {
      return flyWheelRoutine.dynamic(direction);
   }
}
