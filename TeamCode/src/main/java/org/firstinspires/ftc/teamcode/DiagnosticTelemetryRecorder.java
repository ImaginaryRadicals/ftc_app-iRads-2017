package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Utilities.CSV;
import org.firstinspires.ftc.teamcode.Utilities.Color;
import org.firstinspires.ftc.teamcode.Utilities.Constants;

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
    private CSV controlWriter;
    private boolean writeControls = false;


    @Override
    public void init() {
        super.init();
        recordConstantsToFile();
        csvWriter = new CSV(this);
        csvWriter.open("telemetry.csv");
        // Record controls unless dpad down is held during opmode initialization.
        writeControls = !gamepad1.dpad_down;
        if(writeControls) {
            controlWriter = new CSV(this);
            controlWriter.open("controls.csv");
        }
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
        
        telemetry.addData("WRITE CONTROLS",writeControls);
        if(writeControls) {
            writeControlsToFile();
        }
    }

    @Override
    public void stop() {
        super.stop();
        csvWriter.close();
        if(writeControls) {
            controlWriter.close();
        }
    }

    private void recordConstantsToFile() {
        CSV constantsWriter = new CSV(this);
        constantsWriter.open("constants.csv");
        constantsWriter.addFieldToRecord("drive_wheel_diameter", Constants.DRIVE_WHEEL_DIAMETER_INCHES);
        constantsWriter.addFieldToRecord("wheelbase_width_in", Constants.WHEELBASE_WIDTH_IN);
        constantsWriter.addFieldToRecord("wheelbase_length_in", Constants.WHEELBASE_LENGTH_IN);
        constantsWriter.addFieldToRecord("wheelbase_k", Math.abs(Constants.WHEELBASE_LENGTH_IN/2.0)
                                                    + Math.abs(Constants.WHEELBASE_WIDTH_IN/2.0));
        constantsWriter.addFieldToRecord("drive_wheel_steps_per_rotation", (double)Constants.DRIVE_WHEEL_STEPS_PER_ROT);
        constantsWriter.completeRecord();
        constantsWriter.close();
    }

    private void writeControlsToFile() {
        controlWriter.addFieldToRecord("time",time);

        controlWriter.addFieldToRecord("left_stick_x",controller.left_stick_x);
        controlWriter.addFieldToRecord("left_stick_y",controller.left_stick_y);
        controlWriter.addFieldToRecord("right_stick_x",controller.right_stick_x);
        controlWriter.addFieldToRecord("right_stick_y",controller.right_stick_y);
        controlWriter.addFieldToRecord("left_trigger",controller.left_trigger);
        controlWriter.addFieldToRecord("right_trigger",controller.right_trigger);

        controlWriter.addFieldToRecord("right_stick_button",controller.rightStickButton() ? 1.0 : 0.0);
        controlWriter.addFieldToRecord("left_stick_button",controller.leftStickButton() ? 1.0 : 0.0);
        controlWriter.addFieldToRecord("right_bumper",controller.rightBumper() ? 1.0 : 0.0);
        controlWriter.addFieldToRecord("left_bumper",controller.leftBumper() ? 1.0 : 0.0);

        controlWriter.addFieldToRecord("a_button",controller.A() ? 1.0 : 0.0);
        controlWriter.addFieldToRecord("b_button",controller.B() ? 1.0 : 0.0);
        controlWriter.addFieldToRecord("x_button",controller.X() ? 1.0 : 0.0);
        controlWriter.addFieldToRecord("y_button",controller.Y() ? 1.0 : 0.0);

        controlWriter.completeRecord();
    }
}
