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
    private Vector<Vector<Double>> dataTable = new Vector<>();
    private Vector<String> titles = new Vector<>();

    private PrintWriter writer;
    private Vector<Double> currentRecord = new Vector<>();
    private int numRecordsWritten = 0;
    boolean isTitlesPopulated = false;
    boolean isTitlesWritten = false;
    boolean isOpen = false;

    public CSV(RobotHardware robotHardware)
    {
        this.robotHardware = robotHardware;
    }

    public void addRecord(Vector<Double> new_data_record)
    {
        dataTable.add(new_data_record);
    }
    public void addRecord(double[] new_data_record)
    {
        Vector<Double> vec = new Vector<>();
        for (double value : new_data_record)
            vec.add(value);
        dataTable.add(vec);
    }

    public void open(String fileName) {
        try{
            FileOutputStream outputStream = robotHardware.hardwareMap.appContext.openFileOutput(fileName, Context.MODE_PRIVATE);
            writer = new PrintWriter(outputStream);
            isOpen = true;
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    public void addFieldToRecord(String title, Double dataField) {
        currentRecord.add(dataField);
        if(!isTitlesPopulated) {
            titles.add(title);
        }
    }

    public void completeRecord() {
        isTitlesPopulated = true;
        addRecord((Vector<Double>)currentRecord.clone());
        currentRecord.clear();
        write();
    }

    private void write() {
        try{
            // Exit write() if file is not open.
            if(isOpen) {
                // Write titles once
                if(isTitlesPopulated && !isTitlesWritten) {
                    for (int i = 0; i < titles.size(); ++i) {
                        writer.print(titles.get(i));
                        if (i < titles.size() - 1)
                            writer.print(", ");
                    }
                    if (titles.size() > 0) {
                        writer.println();
                    }
                    isTitlesWritten = true; // Write once
                }

                // After Titles are written to file, begin writing records.
                if(isTitlesWritten) {
                    int n_records = dataTable.size();
                    for (int i = numRecordsWritten; i < n_records; ++i) {
                        int n_fields = dataTable.firstElement().size();
                        for (int j = 0; j < n_fields; ++j) {
                            writer.print(dataTable.get(i).get(j));
                            if (j < n_fields - 1)
                                writer.print(", ");
                        }
                        writer.println();
                        ++numRecordsWritten;
                    }
                }
            }
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    public void close() {
        try{
            write();
            writer.close();
            isOpen = false;
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    public void csvWriteAndClose(String fileName)
    {
        csvWriteAndClose(fileName, new Vector<String>());
    }

    public void csvWriteAndClose(String fileName, Vector<String> titles)
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

            int n_steps = dataTable.size();
            for (int i = 0; i < n_steps; ++i)
            {
                int n_states = dataTable.firstElement().size();
                for (int j = 0; j < n_states; ++j)
                {
                    writer.print(dataTable.get(i).get(j));
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
