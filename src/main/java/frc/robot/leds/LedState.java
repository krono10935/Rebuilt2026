package frc.robot.leds;

import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.wpilibj.util.Color;
import java.nio.ByteBuffer;
import java.nio.charset.StandardCharsets;

/**
 * Creates an LED state
 * @param pattern the name of the pattern you want to use
 * @param mainColor The primary color of the pattern
 * @param secondaryColor The seconder color of the pattern (like a background)
 * @param hz how many times does the pattern repeat itself in a second.
 * @param brightness how bright is the pattern (0-1)
 * @param start where does the pattern start on the strip
 * @param end where does the pattern end of the strip
 * @param timeout how much time should the pattern be alive for, 0 is for always, any other value will cancel after the set amount
 */
public record LedState(String pattern, Color mainColor, Color secondaryColor, 
                        double hz, double brightness, int start, int end, double timeout) {

    /**
     * Creates an LED state
     * @param pattern the name of the pattern you want to use
     * @param mainColor The primary color of the pattern
     * @param secondaryColor The seconder color of the pattern (like a background)
     * @param hz how many times does the pattern repeat itself in a second.
     * @param brightness how bright is the pattern (0-1)
     * @param location Where is the pattern
     */
    public LedState(LedPattern pattern, Color mainColor, Color secondaryColor, double hz, double brightness, LedLocation location){
        this(pattern.toString(), mainColor, secondaryColor, hz, brightness, location.start, location.end, 0);
    }

    /**
     * Creates an LED state
     * @param pattern the name of the pattern you want to use
     * @param mainColor The primary color of the pattern
     * @param secondaryColor The seconder color of the pattern (like a background)
     * @param hz how many times does the pattern repeat itself in a second.
     * @param brightness how bright is the pattern (0-1)
     * @param location Where is the pattern
     * @param timeout how much time should the pattern be alive for, 0 is for always, any other value will cancel after the set amount
     */

    public LedState(LedPattern pattern, Color mainColor, Color secondaryColor, double hz, double brightness, LedLocation location, double timeout){
        this(pattern.toString(), mainColor, secondaryColor, hz, brightness, location.start, location.end, timeout);
    }


    // DO NOT TOUCH IT'S BLACK MAGIC
    public static final Struct<LedState> struct = new Struct<>() {
        private static final int STR_MAX = 16;

        @Override
        public Class<LedState> getTypeClass() { return LedState.class; }

        @Override
        public String getTypeString() { return "struct:LedState"; }

        @Override
        public int getSize() {
            // 16 (string) + 24 (Color 1: 3 doubles) + 24 (Color 2: 3 doubles) 
            // + 8 (hz) + 8 (bright) + 4 (start) + 4 (end) + 8 (timeout)
            return STR_MAX + (kSizeDouble * 9) + (kSizeInt32 * 2);
        }

        @Override
        public String getSchema() {
            return "char pattern[" + STR_MAX + "]; " +
                   "double r1; double g1; double b1; " +
                   "double r2; double g2; double b2; " +
                   "double hz; double brightness; " +
                   "int32 start; int32 end; double timeout;";
        }

        @Override
        public void pack(ByteBuffer bb, LedState value) {
            // 1. Pack String (exactly 16 bytes)
            byte[] strBytes = value.pattern().getBytes(StandardCharsets.UTF_8);
            for (int i = 0; i < STR_MAX; i++) {
                bb.put(i < strBytes.length ? strBytes[i] : (byte) 0);
            }

            // 2. Pack Colors (as 3 doubles each)
            bb.putDouble(value.mainColor().red);
            bb.putDouble(value.mainColor().green);
            bb.putDouble(value.mainColor().blue);

            bb.putDouble(value.secondaryColor().red);
            bb.putDouble(value.secondaryColor().green);
            bb.putDouble(value.secondaryColor().blue);

            // 3. Pack remaining primitives
            bb.putDouble(value.hz());
            bb.putDouble(value.brightness());
            bb.putInt(value.start());
            bb.putInt(value.end());
            bb.putDouble(value.timeout());
        }

        @Override
        public LedState unpack(ByteBuffer bb) {
            // 1. Unpack String
            byte[] strBytes = new byte[STR_MAX];
            bb.get(strBytes);
            String pattern = new String(strBytes, StandardCharsets.UTF_8).trim();

            // 2. Unpack Colors
            Color mainColor = new Color(bb.getDouble(), bb.getDouble(), bb.getDouble());
            Color secondaryColor = new Color(bb.getDouble(), bb.getDouble(), bb.getDouble());

            // 3. Unpack Primitives
            return new LedState(
                pattern, mainColor, secondaryColor,
                bb.getDouble(), bb.getDouble(), bb.getInt(), bb.getInt(), bb.getDouble()
            );
        }

        @Override
        public String getTypeName() {
            return "LedState";
        }
    };
}