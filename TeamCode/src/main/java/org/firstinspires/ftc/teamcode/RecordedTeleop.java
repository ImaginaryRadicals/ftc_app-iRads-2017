package org.firstinspires.ftc.teamcode;

import android.content.Context;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Utilities.BlackBox;

import java.io.FileOutputStream;

/**
 * Recorded teleop mode.
 * This mode records the hardware which can later be played back in autonomous.
 * Select the manual control mode by changing the parent class.
 *
 * Thanks to Phillip Tischler  http://pmtischler-ftc-app.readthedocs.io/en/latest/
 */
@TeleOp(name="pmt.Recorded", group="pmtischler")
public class RecordedTeleop extends Manual {
    /**
     * Extends teleop initialization to start a recorder.
     */
    public void init() {
        super.init();
        try {
            telemetry.addData("fileLocation",   hardwareMap.appContext.getFilesDir() );

            outputStream = hardwareMap.appContext.openFileOutput("recordedTeleop",
                                                                 Context.MODE_PRIVATE);
            recorder = new BlackBox.Recorder(hardwareMap, outputStream);
        } catch (Exception e) {
            e.printStackTrace();
            requestOpModeStop();
        }
    }

    /**
     * Extends teleop control to record hardware after loop.
     */
    public void loop() {
        super.loop();

        try {
            for (MotorName m : MotorName.values()) {
                recorder.record(m.name(), time);
            }
            for (ServoName s : ServoName.values()) {
                recorder.record(s.name(), time);
            }
        } catch (Exception e) {
            e.printStackTrace();
            requestOpModeStop();
        }
    }

    /**
     * Closes the file to flush recorded data.
     */
    public void stop() {
        super.stop();

        try {
            outputStream.close();
            telemetry.addData("fileLocation",   hardwareMap.appContext.getFilesDir() );
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    // The output file stream.
    private FileOutputStream outputStream;
    // The hardware recorder.
    private BlackBox.Recorder recorder;
}
