package frc.robot;

public class Ramp {
    private int setpoint = 0;
    private int output = 0;
    
    public void applySetpoint(int setpoint) {
        this.setpoint = setpoint;
    }

    private static final int INCREMENT = 100;

    public void update() {
        if (output - setpoint < -INCREMENT) {
            output += INCREMENT;
        } else if (output - setpoint > INCREMENT) {
            output -= INCREMENT;
        } else {
            output = setpoint;
        }
    }

    public int getOutput() {
        return output;
    }
}