package org.firstinspires.ftc.teamcode.legacy.lib.motion;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.legacy.lib.util.MathFx;

import java.util.ArrayList;

public class TrapezoidalMotionProfile extends Profile {
    private double position, velocity, acceleration;
    private double maxVelocity, maxAcceleration;
    private Double[] positions, velocities, accelerations;
    private ArrayList<Double> positions_temp, velocities_temp, accelerations_temp;
    private double error, dt, direction, set_position;

    public TrapezoidalMotionProfile(double set_position) {
        setSet_position(set_position);
        setKinematics(0 ,0, 0);
        resetLists();
        setLimits(100, 300);
        setDt(0.01);
        setError(set_position);
        setDirection();
        generateProfile(set_position);
    }

    @Override
    public Double[] generateProfile(double set_position) {
        while (Math.abs(error) < 0.05) {
            double output_acceleration;
            double output_velocity;
            if (maxVelocity > Math.abs(velocity)) {
                output_acceleration = maxAcceleration * direction;
                output_velocity = velocity + output_acceleration * dt;
            } else {
                output_acceleration = 0;
                output_velocity = maxVelocity * direction;
            }

            if (Math.abs(error) <= ((output_velocity * output_velocity)/(2 * maxAcceleration))) {
                output_acceleration = -maxAcceleration * direction;
                output_velocity = velocity + output_acceleration * dt;
            }

            output_velocity = MathFx.scale(-maxVelocity, maxVelocity, output_velocity);
            output_acceleration = MathFx.scale(-maxAcceleration, maxAcceleration, output_acceleration);
            run(output_acceleration);
            positions_temp.add(position);
            velocities_temp.add(velocity);
            accelerations_temp.add(acceleration);
            error = set_position - position;
        }
        output();
        return getVelocities();
    }

    public void run(DcMotorEx[] drivers) {
        super.run(getVelocities(), drivers);
    }

    public void setSet_position(double set_position) {
        this.set_position = set_position;
    }

    public void setKinematics(double position, double velocity, double acceleration) {
        this.position = position;
        this.velocity = velocity;
        this.acceleration = acceleration;
    }

    public void resetLists() {
        positions_temp = new ArrayList<Double>();
        velocities_temp = new ArrayList<Double>();
        accelerations_temp = new ArrayList<Double>();
        positions_temp.add(position);
        velocities_temp.add(velocity);
        accelerations_temp.add(acceleration);
    }

    public void setDt(double dt) {
        this.dt = dt;
    }

    public void setLimits(double maxVelocity, double maxAcceleration) {
        this.maxVelocity = maxVelocity;
        this.maxAcceleration = maxAcceleration;
    }

    public void setDirection() {
        direction = 1;
        if (error < 0) {
            direction = -1;
        }
    }

    public void setError(double error) {
        this.error = error;
    }

    public void run(double A) {
        acceleration = MathFx.scale(-maxAcceleration, maxAcceleration, A);
        velocity = MathFx.scale(-maxVelocity, maxVelocity, velocity + acceleration * dt);
        position += velocity * dt;
    }

    public void output() {
        positions = positions_temp.toArray(positions);
        velocities = velocities_temp.toArray(velocities);
        accelerations = accelerations_temp.toArray(accelerations);
    }

    public Double[] getVelocities() {
        return velocities;
    }

}
