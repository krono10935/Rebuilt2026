package frc.utils;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;

import java.io.File;
import java.util.Arrays;

public class ModeFileHandling {
    private static boolean fmsAttached = false;
    private static final String MODE_FILE_PATH = "/home/lvuser/isComp.txt";
    private static final String PIT_COMPUTER_IP = "10.109.35.67";

    /**
     *
     * @return if file for comp mode exists
     */
    public static boolean isCompMode(){
        File file = new File(MODE_FILE_PATH);
        return file.exists();
    }
    /**
     * deletes the comp mode file
     */
    public static void switchToPitMode() {
        File file = new File(MODE_FILE_PATH);
        if(!file.delete()) throw new RuntimeException("Unable to delete pitmode file");

    }

    /**
     *
     * @return if robot should switch to pit mode
     */
    public static boolean shouldSwitchToPitMode() {

        if(fmsAttached) return false;

        fmsAttached = DriverStation.isFMSAttached();

        return Arrays.stream(NetworkTableInstance.getDefault()
        .getConnections()).anyMatch(
            (connection) -> connection.remote_ip.equals(PIT_COMPUTER_IP));
    }

}
