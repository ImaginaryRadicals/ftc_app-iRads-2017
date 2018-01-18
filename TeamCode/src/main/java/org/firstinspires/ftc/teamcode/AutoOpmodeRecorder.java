package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Utilities.CSV;
import org.firstinspires.ftc.teamcode.Utilities.Color;
import org.firstinspires.ftc.teamcode.Utilities.Constants;

/**
 * Created by Ashley on 1/16/2018.
 */

public class AutoOpmodeRecorder extends AutoDeluxe {


    @Autonomous(name="recorder.Red.Center", group="recordAuto")
    public static class AutoRedCenter extends AutoOpmodeRecorder {
        @Override public void init() {
            robotColor = Color.Ftc.RED;
            robotStartPos = StartPosition.FIELD_CENTER;
            super.init();
        }
    }

    @Autonomous(name="recorder.Red.Corner", group="recordAuto")
    public static class AutoRedCorner extends AutoOpmodeRecorder {
        @Override public void init() {
            robotColor = Color.Ftc.RED;
            robotStartPos = StartPosition.FIELD_CORNER;
            super.init();
        }
    }

    @Autonomous(name="recorder.Blue.Center", group="recordAuto")
    public static class AutoBlueCenter extends AutoOpmodeRecorder {
        @Override public void init() {
            robotColor = Color.Ftc.BLUE;
            robotStartPos = StartPosition.FIELD_CENTER;
            super.init();
        }
    }

    @Autonomous(name="recorder.Blue.Corner", group="recordAuto")
    public static class AutoBlueCorner extends AutoOpmodeRecorder {
        @Override public void init() {
            robotColor = Color.Ftc.BLUE;
            robotStartPos = StartPosition.FIELD_CORNER;
            super.init();
        }
    }


    private CSV csvWriter;

    @Override
    public void init() {
        super.init();
        recordConstantsToFile();
        csvWriter = new CSV(this);
        csvWriter.open("telemetry.csv");
    }

    @Override
    public void init_loop() {
        super.init_loop();
    }

    @Override
    public void start() {
        super.start();
        createBlankControlerFile();
    }

    @Override
    public void loop() {
        super.loop();
        addTelemetryToFile();
    }

    @Override
    public void stop() {
        super.stop();
        csvWriter.close();
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
        // Autonomous start position data
        constantsWriter.addFieldToRecord("auto_teamcolor_blue", (double) (this.robotColor == Color.Ftc.BLUE ? 1 : 0));
        constantsWriter.addFieldToRecord("auto_startposition_center", (double) (this.robotStartPos == StartPosition.FIELD_CENTER ? 1 : 0));
        constantsWriter.addFieldToRecord("TeamColor_"+robotColor.name(), (double) 1);
        constantsWriter.addFieldToRecord("StartPosition_"+robotStartPos.name(), (double) 1);
        constantsWriter.completeRecord();
        constantsWriter.close();
    }

    private void addTelemetryToFile() {
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

    private void createBlankControlerFile() {
        CSV controlWriter = new CSV(this);
        controlWriter.open("controls.csv");
        String[] fieldList = {"time", "left_stick_x", "left_stick_y", "right_stick_x",
                "right_stick_y", "left_trigger", "right_trigger", "right_stick_button",
                "left_stick_button", "right_bumper", "left_bumper", "a_button", "b_button",
                "x_button", "y_button"};
       for(int i = 0; i < fieldList.length; ++i) {
           controlWriter.addFieldToRecord(fieldList[i],0.0);
       }
        controlWriter.completeRecord();
    }

}
