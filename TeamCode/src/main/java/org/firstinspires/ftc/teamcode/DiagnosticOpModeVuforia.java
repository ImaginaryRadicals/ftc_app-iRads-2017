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

    private SimpleVuforia vuforia;
    public int msStuckDetectInit = 10000;

    @Override
    public void init() {
        super.init();
        vuforia = new SimpleVuforia(getVuforiaLicenseKey(), this, true, false);
        telemetry.addData("Diagnostic Vuforia Mode ", " Initialized");
    }

    @Override
    public void loop() {
        RelicRecoveryVuMark vuMark = vuforia.detectMark();
        telemetry.addData("Vuforia Glyph Position", vuMark);
        vuforia.displayVuMarkPose();
        super.loop();
    }

}
