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
    public SimpleVuforia(String vuforiaLicenseKey, RobotHardware opMode, boolean useCameraMonitor, boolean useBackCamera) {
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
        if (useBackCamera) {
            parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        } else {
            parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
        }
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

    /**
     * Display the pose of the vuMark, if detected.
     */
    public void displayVuMarkPose() {
        RelicRecoveryVuMark vuMark = detectMark();
        if (vuMark != RelicRecoveryVuMark.UNKNOWN) {

                /* Found an instance of the template. In the actual game, you will probably
                 * loop until this condition occurs, then move on to act accordingly depending
                 * on which VuMark was visible. */
            opMode.telemetry.addData("VuMark", "%s visible", vuMark);

                /* For fun, we also exhibit the navigational pose. In the Relic Recovery game,
                 * it is perhaps unlikely that you will actually need to act on this pose information, but
                 * we illustrate it nevertheless, for completeness. */
            OpenGLMatrix pose = ((VuforiaTrackableDefaultListener)relicTemplate.getListener()).getPose();
            opMode.telemetry.addData("Pose", format(pose));

                /* We further illustrate how to decompose the pose into useful rotational and
                 * translational components */
            if (pose != null) {
                VectorF trans = pose.getTranslation();
                Orientation rot = Orientation.getOrientation(pose, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

                // Extract the X, Y, and Z components of the offset of the target relative to the robot
                double tX = trans.get(0);
                double tY = trans.get(1);
                double tZ = trans.get(2);

                // Extract the rotational components of the target relative to the robot
                double rX = rot.firstAngle;
                double rY = rot.secondAngle;
                double rZ = rot.thirdAngle;
            }
        }
        else {
            opMode.telemetry.addData("VuMark", "not visible");
            opMode.telemetry.addLine();
            opMode.telemetry.addLine();
        }
    }

    String format(OpenGLMatrix transformationMatrix) {
        return (transformationMatrix != null) ? transformationMatrix.formatAsTransform() : "null";
    }

    // The external Vuforia ID localizer.
    private VuforiaLocalizer vuforia;
    private VuforiaTrackables relicTrackables;
    private VuforiaTrackable relicTemplate;
    // opMode reference
    private RobotHardware opMode;
}
