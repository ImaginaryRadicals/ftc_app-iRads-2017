package org.firstinspires.ftc.teamcode.Vision;

import com.qualcomm.robotcore.robot.Robot;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.RobotHardware;

/**
 * Reads Vuforia markers off the camera.
 * Requires the Vuforia key is set in SharedCode/src/main/res/values/vuforia.xml.
 * Example:
 *   private SimpleVuforia vuforia;
 *
 *   public void init() {
 *       String vuforiaKey = "...";
 *       vuforia = new SimpleVuforia(vuforiaKey);
 *   }
 *
 *   public void loop() {
 *       RelicRecoveryVuMark vuMark = vuforia.detectMark();
 *       if (vuMark != RelicRecoveryVuMark.UNKNOWN) {
 *           // ... use detected mark.
 *       }
 *   }
 *
 * Thanks to Phillip Tischler  http://pmtischler-ftc-app.readthedocs.io/en/latest/
 */
public class SimpleVuforia {
    /**
     * Creates a Vuforia localizer and starts localization.
     * @param vuforiaLicenseKey The license key to access Vuforia code.
     */
    public SimpleVuforia(String vuforiaLicenseKey, RobotHardware opMode, boolean useCameraMonitor) {
        this.opMode = opMode;
        VuforiaLocalizer.Parameters parameters;
        if (useCameraMonitor) {
            // Show camera
            int cameraMonitorViewId = opMode.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", opMode.hardwareMap.appContext.getPackageName());
            parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        } else { // Don't use camera monitor
            parameters = new VuforiaLocalizer.Parameters();
        }
        parameters.vuforiaLicenseKey = vuforiaLicenseKey;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        vuforia = ClassFactory.createVuforiaLocalizer(parameters);
        relicTrackables = vuforia.loadTrackablesFromAsset("RelicVuMark");
        relicTemplate = relicTrackables.get(0);
        relicTrackables.activate();
    }

    /**
     * Returns the last detected mark type.
     */
    public RelicRecoveryVuMark detectMark() {
        return RelicRecoveryVuMark.from(relicTemplate);
    }


    // The external Vuforia ID localizer.
    private VuforiaLocalizer vuforia;
    private VuforiaTrackables relicTrackables;
    private VuforiaTrackable relicTemplate;
    // opMode reference
    private RobotHardware opMode;
}
