package frc.robot;

public class PitchYawAdjuster {
    public static double GetYawFromPitch(double pitch) {
        return -1.02254 * pitch - 5.42563;
    }
}