package org.firstinspires.ftc.teamcode.Utilities;

import java.util.Vector;

/**
 * Created by HomeStephen on 12/14/17.
 */

public class Damping {

    private RollingAverage average = new RollingAverage();
    private Dynamics dynamics;

    Damping(Dynamics dynamics) {
        this.dynamics = dynamics;

        average.samples = 50;
    }

    void update(double time, double signal) {
        dynamics.update(time, signal);
        average.update(signal);
    }

    double get() {
        if (dynamics.velocity() > 0.0 && dynamics.acceleration() > 0.0) {
            return average.getAverage();
        }
        else if (dynamics.velocity() < 0.0 && dynamics.acceleration() < 0.0) {
            return average.getAverage();
        }

        return dynamics.signal();
    }
}
