package org.firstinspires.ftc.teamcode.lib.motion;

import org.firstinspires.ftc.teamcode.lib.drivers.Motor;
import org.firstinspires.ftc.teamcode.lib.util.MathFx;
import org.firstinspires.ftc.teamcode.team18103.src.Constants;

public class TrapezoidalMotionProfileGenerator extends Profile {

    private double maxV, maxA;
    private double[] positions, velocities, accelerations;
    private double error, direction, dt, Vt;
    private double tA, tC, tT;
    private int timeSteps;

    public TrapezoidalMotionProfileGenerator(double set_position) {
        setLimits(Motor.GoBILDA_312.maxAngularVelocity(), 300d);
        setError(set_position);
        setDirection();
        setDt(Constants.Dt);
        setTime();
        setLists();
        generateProfile();
    }

    public TrapezoidalMotionProfileGenerator(double set_position, Motor motor) {
        setLimits(motor.maxAngularVelocity(), 300d);
        setError(set_position);
        setDirection();
        setDt(Constants.Dt);
        setTime();
        setLists();
        generateProfile();
    }

    @Override
    public void generateProfile() {
        for (int i = 0; i < getTimeSteps(); i++) {
            double timeStamp = i * dt;
            if (timeStamp < tA) {
                accelerations[i] = (maxA * direction);
                velocities[i] = (MathFx.scale(-maxV, maxV, maxA * timeStamp * direction));
                positions[i] = (maxA * timeStamp * direction * timeStamp / 2);
            } else if (timeStamp < tA + tC) {
                accelerations[i] = (0);
                velocities[i] = (Vt);
                positions[i] = ((Vt * tA) / 2 + (Vt * (timeStamp - tA)));
            } else if (timeStamp < tT) {
                accelerations[i] = (-maxA * direction);
                velocities[i] = (MathFx.scale(-maxV, maxV, Vt - (maxA * direction *
                        (timeStamp - (tA + tC)))));
                positions[i] = ((Vt * tA) / 2 + (Vt * tC) + ((Vt * tA) - (tT - timeStamp) *
                        (Vt - maxA * direction * (timeStamp - (tA + tC)))) / 2);
            } else {
                accelerations[i] = (0);
                velocities[i] = (0);
                positions[i] = (error);
            }
        }
    }

    public void setError(double error) {
        this.error = error;
    }

    public void setDirection() {
        direction = 1;
        if (error < 0) {
            direction = -1;
        }
    }

    public void setDt(double dt) {
        this.dt = dt;
    }

    public void setLimits(double maxV, double maxA) {
        this.maxV = maxV;
        this.maxA = maxA;
    }

    public void setTime() {
        Vt = maxV * direction;
        tA = Vt/maxA * direction;
        tC = (error/Vt) - (maxV/maxA);

        if (tC < 0) {
            Vt = Math.sqrt(error * maxA * direction) * direction;
            tA = Vt / maxA * direction;
            tC = 0;
        }

        tT = 2*tA + tC;

        timeSteps = (int)Math.ceil(tT/dt);
    }

    public int getTimeSteps() {
        return timeSteps;
    }

    public void setLists() {
        positions = new double[timeSteps];
        velocities = new double[timeSteps];
        accelerations = new double[timeSteps];
    }

    public double getTotalTime() {
        return tT;
    }

    public double getMaxV() {
        return maxV;
    }

    @Override
    public double getAcceleration(double timeStamp) {
        timeStamp = MathFx.scale(0, timeStamp, timeStamp);
        int timeStep = (int)(timeStamp/dt);
        if (timeStep < accelerations.length) {
            return accelerations[timeStep];
        }

        return 0d;
    }

    @Override
    public double getVelocity(double timeStamp) {
        timeStamp = MathFx.scale(0, timeStamp, timeStamp);
        int timeStep = (int)(timeStamp/dt);
        if (timeStep < velocities.length) {
            return velocities[timeStep];
        }

        return 0d;
    }

    @Override
    public double getPosition(double timeStamp) {
        timeStamp = MathFx.scale(0, timeStamp, timeStamp);
        int timeStep = (int)(timeStamp/dt);
        if (timeStep < positions.length) {
            return positions[timeStep];
        }

        return error;
    }

}
