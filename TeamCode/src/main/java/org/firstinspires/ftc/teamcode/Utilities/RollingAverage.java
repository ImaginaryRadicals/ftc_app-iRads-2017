package org.firstinspires.ftc.teamcode.Utilities;

import java.util.Vector;

/**
 * Created by ryderswan on 12/7/17.
 */

public class RollingAverage {
    private Vector<Double> signal = new Vector<Double>();
    private double sum = 0.0;
    private double average = 0.0;

    public int samples = 10;

    double update(Double current_value)
    {
        signal.add(current_value);
        sum += current_value;

        while (signal.size() > samples) {
            // remove oldest value
            sum -= signal.get(0);
            signal.remove(0);
        }

        return average = sum / signal.size();
    }

    double getAverage() {
        return average;
    }
}
