package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.teamcode.Utilities.Color;
import org.firstinspires.ftc.teamcode.Vision.SimpleVuforia;


/**
 * Created by Ashley on 12/14/2017.
 *
 * Tests
 */

@TeleOp(name="Diagnostic VuforiaGlyph", group="diagnostic")
public class DiagnosticOpModeVuforia extends DiagnosticOpMode {

    private SimpleVuforia vuforia = null;
    public int msStuckDetectInit = 10000;
    private Thread thread;

    @Override
    public void init() {
        super.init();
        thread = new Thread(new VuforiaLoader());
        thread.start();
    }

    @Override
    public void init_loop() {
        if (vuforia == null) {
            telemetry.addData("Diagnostic Vuforia Mode ", " LOADING");
        } else {
            telemetry.addData("Diagnostic Vuforia Mode ", " INITIALIZED");
        }
    }

    @Override
    public void loop() {
        try {
            RelicRecoveryVuMark vuMark = vuforia.detectMark();
            telemetry.addData("Vuforia Glyph Position", vuMark);
            vuforia.displayVuMarkPose();
        } catch (Exception e) {
            telemetry.addData("Diagnostic Vuforia Mode ", " NOT INITIALIZED");
        }
        super.loop();
    }


    // Initialize vuforia in a separate thread to avoid init() hangups.
    class VuforiaLoader implements Runnable {
        public void run() {
            vuforia = new SimpleVuforia(getVuforiaLicenseKey(), DiagnosticOpModeVuforia.this, true, false);
        }
    }

}
