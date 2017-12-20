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
    }

    @Override
    public void start() {
        super.start();
    }

    @Override
    public void loop() {
        super.loop();
        recordData.clear(); // Reset record fields for this loop iteration.
        // setFieldData sets both titles and recordData.
        setFieldData("time",time);
        setFieldData("red_channel",(double)getColorSensor(ColorSensorName.JEWEL_COLOR, Color.Channel.RED));
        setFieldData("blue_channel",(double)getColorSensor(ColorSensorName.JEWEL_COLOR, Color.Channel.BLUE));
        setFieldData("light_level",(double)getOpticalDistanceSensorLightLevel(OpticalDistanceSensorName.ODS_RIGHT));
        // Capture all servo positions:
        for (ServoName s : ServoName.values()) {
            setFieldData(s.name(), getAngle(s));
        }
        // Capture all motor encoder values:
        for (MotorName m : MotorName.values()) {
            setFieldData(m.name()+"_ticks", getEncoderValue(m));
        }
        // Capture all motor power levels:
        for (MotorName m : MotorName.values()) {
            setFieldData(m.name()+"_power", getPower(m));
        }
        // Capture mecanumNavigation current position
        setFieldData("x_in",mecanumNavigation.currentPosition.x);
        setFieldData("y_in",mecanumNavigation.currentPosition.y);
        setFieldData("theta_rad",mecanumNavigation.currentPosition.theta);

        csvWriter.add((Vector<Double>) recordData.clone());
        isTitleVectorInitialized = true;
    }

    @Override
    public void stop() {
        super.stop();
        csvWriter.csv(new String("telemetry.csv"), recordTitles);
    }

    /**
     * Sets field titles and data in one place.
     * isTitleVectorInitialized must be set to true at the end of the loop.
     * @param fieldTitle
     * @param fieldData
     */
    private void setFieldData(String fieldTitle, double fieldData) {
        if (isTitleVectorInitialized == false) {
            recordTitles.add(fieldTitle);
        }
        recordData.add(fieldData);
    }

}
