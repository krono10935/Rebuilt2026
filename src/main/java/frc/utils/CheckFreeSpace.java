package frc.utils;

import java.io.File;

public class CheckFreeSpace {
    public static double checkUsedPercentage(){
        File drive = new File("D:\\");
        long total = drive.getTotalSpace();
        if (total == 0){
            return 100; // 100% usage: no disk
        }

        long usable = drive.getUsableSpace();
        long used = total - usable;
        double percentUsed = (double) used / total * 100;
        return percentUsed; 
    }
}
