package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Utilities.Color;
import org.firstinspires.ftc.teamcode.Utilities.Mecanum;
import org.firstinspires.ftc.teamcode.Utilities.VectorMath;
import org.firstinspires.ftc.teamcode.Utilities.Constants;

import java.util.ArrayList;
import java.util.Vector;

/**
 *
 * Hardware Abstraction Layer for Robot.
 * This class can be used to define all the specific hardware for a single robot.
 * Thanks to Phillip Tischler  http://pmtischler-ftc-app.readthedocs.io/en/latest/
 */
public abstract class RobotHardware extends OpMode {


    // All motors on the robot, in order of MotorName.
    private ArrayList<DcMotor> allMotors;
    // All servos on the robot, in order of ServoName.
    private ArrayList<Servo> allServos;
    // All optical distance sensors on the robot, in order of DistanceSensorName.
    private ArrayList<OpticalDistanceSensor> allOpticalDistanceSensors;
    // All color sensors on the robot, in order of ColorSensorName.
    private ArrayList<ColorSensor> allColorSensors;

    // Per robot tuning parameters.
    // private String vuforiaLicenseKey;

    // Execution cycle period monitor.
    private ElapsedTime period  = new ElapsedTime();
    private Vector pastPeriods  = new Vector();


    // The motors on the robot.
    public enum MotorName {
        DRIVE_FRONT_LEFT,
        DRIVE_FRONT_RIGHT,
        DRIVE_BACK_LEFT,
        DRIVE_BACK_RIGHT,
        ARM_LIFT,
    }

    // Motor methods

    /**
     * Sets the power of the motor.
     * @param motor The motor to modify.
     * @param power The power to set [-1, 1].
     */
    protected void setPower(MotorName motor, double power) {
        DcMotor m = allMotors.get(motor.ordinal());
        if (m == null) {
            telemetry.addData("Motor Missing", motor.name());
        } else {
            m.setPower(power);
        }
    }

    /**
     * Sets the drive chain power.
     * @param left The power for the left two motors.
     * @param right The power for the right two motors.
     */
    protected void setDriveForTank(double left, double right) {
        setPower(MotorName.DRIVE_FRONT_LEFT, left);
        setPower(MotorName.DRIVE_BACK_LEFT, left);
        setPower(MotorName.DRIVE_FRONT_RIGHT, right);
        setPower(MotorName.DRIVE_BACK_RIGHT, right);
    }

    /**
     * Sets the drive chain power from Mecanum motion.
     * Maintains relative speeds when changing angles.
     * @param motion The desired Mecanum motion.
     */
    protected void setDriveForMecanum(Mecanum.Motion motion) {
        Mecanum.Wheels wheels = Mecanum.motionToWheels(motion);
        setPower(MotorName.DRIVE_FRONT_LEFT, wheels.frontLeft);
        setPower(MotorName.DRIVE_BACK_LEFT, wheels.frontRight);
        setPower(MotorName.DRIVE_FRONT_RIGHT, wheels.backLeft);
        setPower(MotorName.DRIVE_BACK_RIGHT, wheels.backRight);
    }

    /**
     * Sets the drive chain power from Mecanum motion.
     * Uses max power output while changing speeds at angle motions.
     * @param motion The desired Mecanum motion.
     */
    protected void setDriveForMecanumForSpeed(Mecanum.Motion motion) {
        Mecanum.Wheels wheels = Mecanum.motionToWheels(motion).scaleWheelPower(
                Math.sqrt(2));
        setPower(MotorName.DRIVE_FRONT_LEFT, wheels.frontLeft);
        setPower(MotorName.DRIVE_BACK_LEFT, wheels.frontRight);
        setPower(MotorName.DRIVE_FRONT_RIGHT, wheels.backLeft);
        setPower(MotorName.DRIVE_BACK_RIGHT, wheels.backRight);
    }


    // The servos on the robot.
    protected enum ServoName {
        CLAW_LEFT,
        CLAW_RIGHT,
    }

    // Servo methods
    
    /**
     * Sets the angle of the servo.
     * @param servo The servo to modify.
     * @param position The angle to set [0, 1].
     */
    protected void setAngle(ServoName servo, double position) {
        Servo s = allServos.get(servo.ordinal());
        if (s == null) {
            telemetry.addData("Servo Missing", servo.name());
        } else {
            s.setPosition(position);
        }
    }

    // Opens the servo claw
    protected void openClaw() {
        setAngle(ServoName.CLAW_LEFT, Constants.LEFT_CLAW_OPEN);
        setAngle(ServoName.CLAW_RIGHT, Constants.RIGHT_CLAW_OPEN);
    }

    // Closes the servo claw
    protected void closeClaw() {
        setAngle(ServoName.CLAW_LEFT, Constants.LEFT_CLAW_CLOSED);
        setAngle(ServoName.CLAW_RIGHT, Constants.RIGHT_CLAW_CLOSED);
    }

    // Opens the servo claw slighly
    protected void slightOpenClaw() {
        setAngle(ServoName.CLAW_LEFT, Constants.LEFT_CLAW_RELEASE);
        setAngle(ServoName.CLAW_RIGHT, Constants.RIGHT_CLAW_RELEASE);
    }

    // Closes the servo claw
    protected void storeClaw() {
        setAngle(ServoName.CLAW_LEFT, Constants.INITIAL_LEFT_CLAW_POS);
        setAngle(ServoName.CLAW_RIGHT, Constants.INITIAL_RIGHT_CLAW_POS);
    }

    // Set servo claw to given position
    protected void setPositionClaw(double position) {
        setAngle(ServoName.CLAW_LEFT, position);
        setAngle(ServoName.CLAW_RIGHT, position);
    }



    // The Optical Distance Sensors on the robot.
    protected enum OpticalDistanceSensorName {
        ODS_LEFT,
        ODS_RIGHT,
    }


    // The color sensors on the robot.
    protected enum ColorSensorName {
        JEWEL_COLOR,
    }

    /**
     * Gets the color value on the sensor.
     * @param sensor The sensor to read.
     * @param color The color channel to read intensity.
     */
    protected int getColorSensor(ColorSensorName sensor, Color.Channel color) {
        ColorSensor s = allColorSensors.get(sensor.ordinal());
        if (s == null) {
            telemetry.addData("Color Sensor Missing", sensor.name());
            return 0;
        }

        switch (color) {
            case RED: return s.red();
            case GREEN: return s.green();
            case BLUE: return s.blue();
            case ALPHA: return s.alpha();
            default: return 0;
        }
    }

    /**
     * Sets the LED power for the color sensor.
     * @param sensor The sensor to set the LED power.
     * @param enabled Whether to turn the LED on.
     */
    protected void setColorSensorLedEnabled(ColorSensorName sensor,
                                         boolean enabled) {
        ColorSensor s = allColorSensors.get(sensor.ordinal());
        if (s == null) {
            telemetry.addData("Color Sensor Missing", sensor.name());
        } else {
            s.enableLed(enabled);
        }
    }


    // Possible starting positions.
    protected enum StartPosition {
        FIELD_CENTER,
        FIELD_CORNER,
    }

    /**
     * Gets the Vuforia license key.
     */
    protected String getVuforiaLicenseKey() {
    String vuforiaLicenseKey = "AUQ7leT/////AAAAGbE5ttrmO0iOg4xdJTnQehMYDMxLvRqCeEEhtqeZWJzzoNESAE9U6OUW7BmVwUSNmsVtZb1p6ALNdMJnozgpwyLM98L/E2+omz7xJqvSsDqnhlDqFUeoTd4xKyVjcKinMPzkkvFbJHrh9bHWXqvY3Z68QtNbJiiyLLvXuFmk/Y/ZnFBzUT7fZzuQsceQZJVbvmokgb+TRN8Wy+RHRYtOhHznJOVOdxTp2OEHY1nLWwq0trt4ozfzzpu/8Mk2Vym/gKaZk9cyAA0tyduKk5r+6Zx+o/mUPN7Ox5qjhXOaYxz1amH05ieZOPSu8MXSM47L+5WxD4riIfPBY2fjfrFtq4EXyhTo9VjHD0gd1N0cXbaw";
        return vuforiaLicenseKey;
    }


    /**
     * Should be executed at the beginning of loop() function.
     * Adds the most recent period length (in seconds) to a vector.
     * then, calculates the average period length.
     * @return Average period of past execution cycles.
     */
    public double periodSec(){

        pastPeriods.add(period.time());
        period.reset();
        if (pastPeriods.size()>= 30) {
            pastPeriods.remove(0);
        }
        return VectorMath.average(pastPeriods);
    }


    /**
     * Initialize the hardware handles.
     */
    public void init() {
        /*
        vuforiaLicenseKey = hardwareMap.appContext.getResources().getString(
                R.string.vuforia_key);
                */

        allMotors = new ArrayList<DcMotor>();
        for (MotorName m : MotorName.values()) {
            try {
                allMotors.add(hardwareMap.get(DcMotor.class, m.name()));
            } catch (Exception e) {
                telemetry.addData("Motor Missing", m.name());
                allMotors.add(null);
            }
        }
        // Set motor directions.
        try {
            allMotors.get(MotorName.DRIVE_FRONT_RIGHT.ordinal()).setDirection(DcMotor.Direction.REVERSE);
            allMotors.get(MotorName.DRIVE_BACK_RIGHT.ordinal()).setDirection(DcMotor.Direction.REVERSE);
        } catch (Exception e) {
            telemetry.addData("Unable to set right motor direction","");
        }

        allServos = new ArrayList<Servo>();
        for (ServoName s : ServoName.values()) {
            try {
                allServos.add(hardwareMap.get(Servo.class, s.name()));
            } catch (Exception e) {
                telemetry.addData("Servo Missing", s.name());
                allServos.add(null);
            }
        }
        // Set servo direction
        try {
            allServos.get(ServoName.CLAW_LEFT.ordinal()).setDirection(Servo.Direction.REVERSE);
        } catch (Exception e) {
            telemetry.addData("Unable to set left servo direction", "");
        }

        allColorSensors = new ArrayList<ColorSensor>();
        for (ColorSensorName s : ColorSensorName.values()) {
            try {
                allColorSensors.add(hardwareMap.get(ColorSensor.class,
                                                    s.name()));
            } catch (Exception e) {
                telemetry.addData("Color Sensor Missing", s.name());
                allColorSensors.add(null);
            }
        }

        allOpticalDistanceSensors = new ArrayList<OpticalDistanceSensor>();
        for (OpticalDistanceSensorName s : OpticalDistanceSensorName.values()) {
            try {
                allOpticalDistanceSensors.add(hardwareMap.get(
                            OpticalDistanceSensor.class,
                            s.name()));
            } catch (Exception e) {
                telemetry.addData("Optical Distance Sensor Missing", s.name());
                allOpticalDistanceSensors.add(null);
            }
        }

        storeClaw();
    }



    /**
     * End of match, stop all actuators.
     */
    public void stop() {
        super.stop();

        for (MotorName m : MotorName.values()) {
            setPower(m, 0);
        }
        for (ColorSensorName s : ColorSensorName.values()) {
            setColorSensorLedEnabled(s, false);
        }
    }


}

