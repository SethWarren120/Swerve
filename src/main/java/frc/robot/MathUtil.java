package frc.robot;

public class MathUtil {
    /**
     * Clamps a given value to be in the range between two given values
     * @param value The value to clamp
     * @param min The lowest permissible value
     * @param max The highest permissible value
     * @return The clamped value
     */
    public static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    /**
     * Clamps a given value to be in the range between two given values
     * @param value The value to clamp
     * @param min The lowest permissible value
     * @param max The highest permissible value
     * @return The clamped value
     */
    public static int clamp(int value, int min, int max) {
        return Math.max(min, Math.min(max, value));
    }
}
