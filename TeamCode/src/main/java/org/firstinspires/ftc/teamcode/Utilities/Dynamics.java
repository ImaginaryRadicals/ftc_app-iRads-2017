package org.firstinspires.ftc.teamcode.Utilities;

import java.util.Vector;

import static org.firstinspires.ftc.teamcode.Utilities.VectorMath.addRemove3;
import static org.firstinspires.ftc.teamcode.Utilities.VectorMath.derivative;

/**
 * Created by HomeStephen on 12/14/17.
 */

public class Dynamics {

    private Vector<Double> time = new Vector<>();
    private Vector<Double> s = new Vector<>();
    private Vector<Double> v = new Vector<>();
    private Vector<Double> a = new Vector<>();

    void update(double t, double signal)
    {
        addRemove3(time, t);
        addRemove3(s, signal);

        if (s.size() >= 2) {
            addRemove3(v, derivative(time, s));
        }
        if (v.size() >= 2) {
            addRemove3(a, derivative(time, v));
        }
    }

    public Vector<Double> getSignal() {
        return s;
    }

    public Vector<Double> getVelocity() {
        return v;
    }

    public Vector<Double> getAcceleration() {
        return a;
    }

    public Double signal() {
        if (s.size() > 0) {
            return s.lastElement();
        }
        return 0.0;
    }

    public Double velocity() {
        if (v.size() > 0) {
            return v.lastElement();
        }
        return 0.0;
    }

    public Double acceleration() {
        if (a.size() > 0) {
            return a.lastElement();
        }
        return 0.0;
    }
}
