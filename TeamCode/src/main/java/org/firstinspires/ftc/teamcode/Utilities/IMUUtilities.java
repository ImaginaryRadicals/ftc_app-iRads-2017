package org.firstinspires.ftc.teamcode.Utilities;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.robot.Robot;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.teamcode.RobotHardware;

import java.util.Locale;

/**
 * Created by Ashley on 1/18/2018.
 */

public class IMUUtilities {

    // Convenience numbers for when IMUUtilities is actually instantiated.
    public BNO055IMU imu;
    public double heading;
    public double roll;
    public double pitch;
    public double xAccel;
    public double yAccel;
    public double zAccel;

    protected RobotHardware opMode;
    protected Orientation angles;
    protected Acceleration gravity;
    protected Acceleration acceleration;

    public IMUUtilities(RobotHardware opMode, String imu_name) {
        this.opMode = opMode;
        imu = initializeIMU(this.opMode, imu_name);
        startIMU(imu);
    }


    public void update() {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        gravity = imu.getGravity();
        acceleration = imu.getLinearAcceleration();

        heading = angles.firstAngle;
        roll = angles.secondAngle;
        pitch = angles.thirdAngle;
        xAccel = acceleration.xAccel;
        yAccel = acceleration.yAccel;
        zAccel = acceleration.zAccel;
    }

    // Static Functions

    /**
     * Use IMU config name to initialize and return a reference to the imu hardware.
     * @param opMode
     * @param imu_name
     * @return
     */
    static public BNO055IMU initializeIMU(RobotHardware opMode, String imu_name) {
        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "AdafruitIMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        BNO055IMU imu;
        try {
            imu = opMode.hardwareMap.get(BNO055IMU.class, imu_name);
            imu.initialize(parameters);
        } catch (Exception e) {
            imu = null;
            opMode.telemetry.addData("IMU Missing", imu_name);
        }
        return imu;
    }

    /**
     * Start IMU integration and logging.
     * @param imu
     */
    static public void startIMU (BNO055IMU imu) {
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
    }


    /**
     * Display telemetry data for the IMU
     * @param imu
     * @param opMode
     */
    static public void displayIMUTelemetry(final BNO055IMU imu, RobotHardware opMode) {

        // Acquiring the angles is relatively expensive; we don't want
        // to do that in each of the three items that need that info, as that's
        // three times the necessary expense.
        final Orientation angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        final Acceleration gravity  = imu.getGravity();
        final Acceleration acceleration = imu.getLinearAcceleration();

        opMode.telemetry.addLine()
                .addData("status", new Func<String>() {
                    @Override public String value() {
                        return imu.getSystemStatus().toShortString();
                    }
                })
                .addData("calib", new Func<String>() {
                    @Override public String value() {
                        return imu.getCalibrationStatus().toString();
                    }
                });

        opMode.telemetry.addLine()
                .addData("heading", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.firstAngle);
                    }
                })
                .addData("roll", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.secondAngle);
                    }
                })
                .addData("pitch", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.thirdAngle);
                    }
                });

        opMode.telemetry.addLine()
                .addData("grvty", new Func<String>() {
                    @Override public String value() {
                        return gravity.toString();
                    }
                })
                .addData("mag", new Func<String>() {
                    @Override public String value() {
                        return String.format(Locale.getDefault(), "%.3f",
                                Math.sqrt(gravity.xAccel*gravity.xAccel
                                        + gravity.yAccel*gravity.yAccel
                                        + gravity.zAccel*gravity.zAccel));
                    }
                });

        opMode.telemetry.addLine()
                .addData("X Accel", new Func<String>() {
                    @Override public String value() {
                        return String.format(Locale.getDefault(), "%.3f", acceleration.xAccel);
                    }
                })
                .addData("Y Accel", new Func<String>() {
                    @Override public String value() {
                        return String.format(Locale.getDefault(), "%.3f", acceleration.yAccel);
                    }
                })
                .addData("Z Accel", new Func<String>() {
                    @Override public String value() {
                        return String.format(Locale.getDefault(), "%.3f", acceleration.zAccel);
                    }
                });
    }


    static public class OrientationAngles {
        public double heading;
        public double roll;
        public double pitch;

        public OrientationAngles(double heading, double roll, double pitch) {
            this.heading = heading;
            this.roll = roll;
            this.pitch = pitch;
        }
    }


    /**
     * Return object with robot heading, roll, and pitch in degrees.
     * @param imu
     * @return OrientationAngles object
     */
    static public OrientationAngles getOrientation(BNO055IMU imu) {
        final Orientation angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        // Return heading, roll, pitch
        return new OrientationAngles(angles.firstAngle, angles.secondAngle, angles.thirdAngle);
    }

    /**
     * Redundant method, returns Acceleration object with xAccel, yAccel, zAccell components.
     * @param imu
     * @return Acceleration object
     */
    static public Acceleration getAccelerationComponents(BNO055IMU imu) {
        return  imu.getLinearAcceleration();
    }
    //----------------------------------------------------------------------------------------------
    // Formatting
    //----------------------------------------------------------------------------------------------

    static protected String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    static protected String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

}
