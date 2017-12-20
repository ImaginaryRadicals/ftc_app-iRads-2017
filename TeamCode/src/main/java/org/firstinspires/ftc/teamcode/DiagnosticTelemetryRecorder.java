package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Utilities.CSV;
import org.firstinspires.ftc.teamcode.Utilities.Color;

import java.util.Vector;

/**
 * Created by Ashley on 12/19/2017.
 */

@TeleOp(name="Diagnostic Telemetry Recorder", group="diagnostic")
public class DiagnosticTelemetryRecorder extends DiagnosticOpMode {

    private boolean isTitleVectorInitialized = false;
    private Vector<String> recordTitles = new Vector<>();
    private Vector<Double> recordData = new Vector<>();
    private CSV csvWriter;


    @Override
    public void init() {
        super.init();
        csvWriter = new CSV(this);
        csvWriter.open("telemetry.csv");
    }

    @Override
    public void start() {
        super.start();
    }

    @Override
    public void loop() {
        super.loop();

        // setFieldData sets both titles and recordData.
        csvWriter.addFieldToRecord("time",time);
        csvWriter.addFieldToRecord("red_channel",(double)getColorSensor(ColorSensorName.JEWEL_COLOR, Color.Channel.RED));
        csvWriter.addFieldToRecord("blue_channel",(double)getColorSensor(ColorSensorName.JEWEL_COLOR, Color.Channel.BLUE));
        csvWriter.addFieldToRecord("light_level",getOpticalDistanceSensorLightLevel(OpticalDistanceSensorName.ODS_RIGHT));
        // Capture all servo positions:
        for (ServoName s : ServoName.values()) {
            csvWriter.addFieldToRecord(s.name(), getAngle(s));
        }
        // Capture all motor encoder values:
        for (MotorName m : MotorName.values()) {
            csvWriter.addFieldToRecord(m.name()+"_ticks", (double)getEncoderValue(m));
        }
        // Capture all motor power levels:
        for (MotorName m : MotorName.values()) {
            csvWriter.addFieldToRecord(m.name()+"_power", getPower(m));
        }
        // Capture mecanumNavigation current position
        csvWriter.addFieldToRecord("x_in",mecanumNavigation.currentPosition.x);
        csvWriter.addFieldToRecord("y_in",mecanumNavigation.currentPosition.y);
        csvWriter.addFieldToRecord("theta_rad",mecanumNavigation.currentPosition.theta);

        // Writes record to file if writer is open.
        csvWriter.completeRecord();
    }

    @Override
    public void stop() {
        super.stop();
        csvWriter.close();
    }

}
