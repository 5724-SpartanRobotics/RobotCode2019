package frc.robot;

public class SRamp {
    private float setpoint = 0;
    private float output = 0;
    private float currentVel = 0;

    private static final float MAX_VEL = 100;
    private static final float MAX_ACCEL = 1;
    
    public void applySetpoint(float value) {
        setpoint = value;
    }

    public void update() {
        float error = setpoint - output;    
        float diff = currentVel / MAX_ACCEL;

        if (Math.abs(error) <= diff) {
            if (error > 0) {
                currentVel -= diff;
            } else {
                currentVel += diff;
            }
        } else if (currentVel < MAX_VEL) {
            currentVel += MAX_ACCEL;
        }

        output += currentVel;
    }

    public float getOutput() {
        return output;
    }
}