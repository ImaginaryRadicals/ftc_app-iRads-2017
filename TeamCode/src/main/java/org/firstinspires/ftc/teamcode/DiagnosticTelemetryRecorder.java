package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Utilities.CSV;

import java.util.Vector;

/**
 * Created by Ashley on 12/19/2017.
 */

@TeleOp(name="Diagnostic Telemetry Recorder", group="diagnostic")
public class DiagnosticTelemetryRecorder extends DiagnosticOpMode {

    private Vector<String> recordTitles = new Vector<>();
    private CSV csvWriter;


    @Override
    public void init() {
        super.init();
        csvWriter = new CSV(this);
        recordTitles.add(new String("time"));
    }

    @Override
    public void start() {
        super.start();
    }

    @Override
    public void loop() {
        super.loop();
        Vector<Double> recordData = new Vector<>();
        recordData.add(time);
        csvWriter.add(recordData);
    }

    @Override
    public void stop() {
        super.stop();
        csvWriter.csv(new String("telemetry.csv"), recordTitles);
    }

}
