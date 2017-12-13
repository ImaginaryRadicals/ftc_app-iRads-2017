package org.firstinspires.ftc.teamcode.Utilities;

import org.firstinspires.ftc.teamcode.RobotHardware;
import android.content.Context;
import java.io.FileOutputStream;
import java.io.PrintWriter;
import java.util.Vector;

/**
 * Created by HomeStephen on 12/12/17.
 */

public class CSV {
    private RobotHardware robotHardware;
    private Vector<Vector<Double>> data = new Vector<>();
    private Vector<String> titles = new Vector<>();

    CSV(RobotHardware robotHardware)
    {
        this.robotHardware = robotHardware;
    }

    void add(Vector<Double> new_data)
    {
        data.add(new_data);
    }
    void add(double[] new_data)
    {
        Vector<Double> vec = new Vector<>();
        for (double value : new_data)
            vec.add(value);
        data.add(vec);
    }

    void csv(String fileName)
    {
        csv(fileName, new Vector<String>());
    }

    void csv(String fileName, Vector<String> titles)
    {
        try{
            FileOutputStream outputStream = robotHardware.hardwareMap.appContext.openFileOutput(fileName, Context.MODE_PRIVATE);

            PrintWriter writer = new PrintWriter(outputStream);

            for (int i = 0; i < titles.size(); ++i)
            {
                writer.print(titles.get(i));
                if (i < titles.size() - 1)
                    writer.print(", ");
            }
            if (titles.size() > 0)
                writer.println();

            int n_steps = data.size();
            for (int i = 0; i < n_steps; ++i)
            {
                int n_states = data.firstElement().size();
                for (int j = 0; j < n_states; ++j)
                {
                    writer.print(data.get(i).get(j));
                    if (j < n_states - 1)
                        writer.print(", ");
                }
                writer.println();
            }

            writer.close();
        } catch (Exception e) {
            e.printStackTrace();
        }
    }
}
