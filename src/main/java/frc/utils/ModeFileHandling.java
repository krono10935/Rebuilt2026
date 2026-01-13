package frc.utils;

import edu.wpi.first.wpilibj.DriverStation;

import java.io.File;
import java.io.FileNotFoundException;

public class ModeFileHandling {
    private static boolean fmsAttached = false;
    private static final String MODE_FILE_PATH = "/home/lvuser/isComp.txt";

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

        //TODO: set actual check for pit mode
        return true;
    }

}
