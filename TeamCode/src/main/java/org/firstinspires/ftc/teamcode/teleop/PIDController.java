package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * A simple PID controller.
 */
public class PIDController {
    private double kp, ki, kd;
    private double integralSum = 0;
    private double lastError = 0;
    private ElapsedTime timer = new ElapsedTime();

    public PIDController(double kp, double ki, double kd) {
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
    }

    /**
     * Calculates the control output.
     * @param reference the target value
     * @param state the current value
     * @return the control output
     */
    public double calculate(double reference, double state) {
        double error = reference - state;
        double dt = timer.seconds();
        
        // Integral term
        integralSum += error * dt;

        // Derivative term
        double derivative = 0;
        if (dt != 0) {
            derivative = (error - lastError) / dt;
        }

        lastError = error;
        timer.reset();

        return (kp * error) + (ki * integralSum) + (kd * derivative);
    }

    /**
     * Resets the controller's state.
     */
    public void reset() {
        integralSum = 0;
        lastError = 0;
        timer.reset();
    }
}
