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

import java.text.DecimalFormat;
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

    // Names of only the drive motors.
    private ArrayList<MotorName> driveMotorNames;


    // Execution cycle period monitor.
    private ElapsedTime period  = new ElapsedTime();
    private Vector pastPeriods  = new Vector();


    // The motors on the robot.
    public enum MotorName {
        DRIVE_FRONT_LEFT,
        DRIVE_FRONT_RIGHT,
        DRIVE_BACK_LEFT,
        DRIVE_BACK_RIGHT,
        ARM_MOTOR,
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
     * Gets the encoder value of the motor.
     * @param motor MotorName enum value.
     * @return integer encoder position in ticks.
     */
    public int getEncoderValue(MotorName motor) {
        DcMotor m = allMotors.get(motor.ordinal());
        if (m == null) {
            telemetry.addData("Motor Missing", motor.name());
            return 0;
        } else {
            return m.getCurrentPosition();
        }
    }

    /**
     * Stops all motors.
     */
    protected void stopAllMotors() {
        for (MotorName m : MotorName.values()) {
            setPower(m, 0);
        }
    }

    /**
     * Set all four drive motors to the same runMode.
     * Options are RUN_WITHOUT_ENCODER, RUN_USING_ENCODER,
     * RUN_TO_POSITION is used with setTargetPosition()
     * STOP_AND_RESET_ENCODER
     * @param runMode
     */
    protected void setDriveMotorsRunMode(DcMotor.RunMode runMode) {
        for (MotorName motor : driveMotorNames) {
            DcMotor m = allMotors.get(motor.ordinal());
            if (m == null) {
                telemetry.addData("Motor Missing", motor.name());
            } else {
                m.setMode(runMode);
            }
        }
    }

    /**
     * Set zero power braking behavior for all drive wheels.
     * @param zeroPowerBraking boolean to activate or deactivate zero power braking
     */
    protected void setDriveMotorsZeroPowerBraking(boolean zeroPowerBraking) {
        DcMotor.ZeroPowerBehavior brakingMode = zeroPowerBraking ? DcMotor.ZeroPowerBehavior.BRAKE
                : DcMotor.ZeroPowerBehavior.FLOAT;
        for (MotorName motor : driveMotorNames) {
            DcMotor m = allMotors.get(motor.ordinal());
            if (m == null) {
                telemetry.addData("Motor Missing", motor.name());
            } else {
                m.setZeroPowerBehavior(brakingMode);
            }
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
     * Apply motor power matching the wheels object
     * @param wheels Provides all four mecanum wheel powers, [-1, 1]
     */
    protected void setDriveForMecanumWheels(Mecanum.Wheels wheels) {
        setPower(MotorName.DRIVE_FRONT_LEFT, wheels.frontLeft);
        setPower(MotorName.DRIVE_BACK_LEFT, wheels.backLeft);
        setPower(MotorName.DRIVE_FRONT_RIGHT, wheels.frontRight);
        setPower(MotorName.DRIVE_BACK_RIGHT, wheels.backRight);
    }

    /**
     * Sets mecanum drive chain power using simplistic calculations.
     * @param leftStickX Unmodified Gamepad leftStickX inputs.
     * @param leftStickY Unmodified Gamepad leftStickY inputs.
     * @param rightStickX Unmodified Gamepad rightStickX inputs.
     * @param rightStickY Unmodified Gamepad rightStickY inputs.
     */
    protected void setDriveForSimpleMecanum(double leftStickX, double leftStickY,
                                            double rightStickX, double rightStickY) {
        Mecanum.Wheels wheels = Mecanum.simpleJoystickToWheels (leftStickX, leftStickY, rightStickX, rightStickY);
        setDriveForMecanumWheels(wheels);
    }

    /**
     * Sets the drive chain power from Mecanum motion.
     * Maintains relative speeds when changing angles.
     * @param motion The desired Mecanum motion.
     */
    protected void setDriveForMecanum(Mecanum.Motion motion) {
        Mecanum.Wheels wheels = Mecanum.motionToWheels(motion);
        setDriveForMecanumWheels(wheels);
    }

    /**
     * Sets the drive chain power from Mecanum motion.
     * Uses max power output while changing speeds at angle motions.
     * @param motion The desired Mecanum motion.
     */
    protected void setDriveForMecanumForSpeed(Mecanum.Motion motion) {
        Mecanum.Wheels wheels = Mecanum.motionToWheels(motion).scaleWheelPower(
                Math.sqrt(2));
        setDriveForMecanumWheels(wheels);
    }


    // Arm positions
    protected void raiseArm(){
        setTargetPosition(MotorName.ARM_MOTOR, Constants.RAISE_ARM_POSITION);
    }
    protected void lowerArm(){
        setTargetPosition(MotorName.ARM_MOTOR, Constants.LOWER_ARM_POSITION);
    }

    //Same thing as the Servo one, but for a motor
    protected void setTargetPosition(MotorName motor, int position) {
        DcMotor s = allMotors.get(motor.ordinal());
        if (s == null) {
            telemetry.addData("Motor Missing", motor.name());
        } else {
            s.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            s.setPower(1.0);
            s.setTargetPosition(position);
        }
    }


    // The servos on the robot.
    protected enum ServoName {
        CLAW_LEFT,
        CLAW_RIGHT,
        JEWEL_ARM,
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

    /**
     * Get the position of a servo.
     * @param servo ServoName enum to check
     * @return double servo position [0,1]
     */
    protected double getAngle(ServoName servo) {
        Servo s = allServos.get(servo.ordinal());
        if (s == null) {
            telemetry.addData("Servo Missing", servo.name());
            return -1;
        } else {
            return s.getPosition();
        }
    }

    // Sets the Jewel are servo to the init position

    protected void armServoStored() {
        setAngle(ServoName.JEWEL_ARM, Constants.JEWEL_ARM_INTIAL);
    }

    protected  void armServoTop() {
        setAngle(ServoName.JEWEL_ARM, Constants.JEWEL_ARM_TOP);
    }

    protected  void armServoBottom() {
        setAngle(ServoName.JEWEL_ARM, Constants.JEWEL_ARM_BOTTOM);
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

    protected void setPositionJewelArm(double position) {
        setAngle(ServoName.JEWEL_ARM, position);
    }



    // The Optical Distance Sensors on the robot.
    protected enum OpticalDistanceSensorName {
        //ODS_LEFT,
        ODS_RIGHT,
    }


    /**
     * Get the amount of light detected by the optical distance sensor.
     * @param sensor The sensor to read.
     * @return The light level detected on a scale from 0.0 to 1.0.
     */
    protected double getOpticalDistanceSensorLightLevel(OpticalDistanceSensorName sensor) {
        OpticalDistanceSensor ods = allOpticalDistanceSensors.get(sensor.ordinal());
        if (ods == null) {
            telemetry.addData("Optical Distance Sensor Missing", sensor.name());
            return 0;
        } else {
            return ods.getLightDetected();
        }
    }


    /**
     * Sets the LED power for the optical distance sensor.
     * @param sensor The ods to set the LED power.
     * @param enabled Whether to turn the LED on.
     */
    protected void setOpticalDistanceSensorLedEnable(OpticalDistanceSensorName sensor,
                                                     boolean enabled) {
        OpticalDistanceSensor ods = allOpticalDistanceSensors.get(sensor.ordinal());
        if (ods == null) {
            telemetry.addData("Optical Distance Sensor Missing", sensor.name());
        } else {
            ods.enableLed(enabled);
        }
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

    /**
     * Checks JEWEL_COLOR sensor and returns enum based on color detected.
     * @return RED, BLUE, or null
     */
    protected Color.Ftc getJewelColor() {
        int red = getColorSensor(ColorSensorName.JEWEL_COLOR, Color.Channel.RED);
        int blue = getColorSensor(ColorSensorName.JEWEL_COLOR, Color.Channel.BLUE);
        if (red > blue) {
            return Color.Ftc.RED;
        } else if (blue > red) {
            return Color.Ftc.BLUE;
        } else {
            return Color.Ftc.UNKNOWN;
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

        // Collect a list of only the drive motors.
        driveMotorNames = new ArrayList<MotorName>();
            driveMotorNames.add(MotorName.DRIVE_FRONT_LEFT);
            driveMotorNames.add(MotorName.DRIVE_FRONT_RIGHT);
            driveMotorNames.add(MotorName.DRIVE_BACK_LEFT);
            driveMotorNames.add(MotorName.DRIVE_BACK_RIGHT);

        // Set motor directions.
        try {
            allMotors.get(MotorName.DRIVE_FRONT_LEFT.ordinal()).setDirection(DcMotor.Direction.REVERSE);
            allMotors.get(MotorName.DRIVE_BACK_LEFT.ordinal()).setDirection(DcMotor.Direction.REVERSE);
            allMotors.get(MotorName.ARM_MOTOR.ordinal()).setDirection(DcMotor.Direction.FORWARD);
        } catch (Exception e) {
            telemetry.addData("Unable to set motor direction","");
        }

        // Set drive motors to use encoders
        setDriveMotorsRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
        // Set drive motors to float instead of brake when power is zero.
        setDriveMotorsZeroPowerBraking(false);

        // Set arm motor to brake
        try {
            allMotors.get(MotorName.ARM_MOTOR.ordinal()).setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        } catch (Exception e){
            telemetry.addData("Unable to set arm motor to zero power brake", "");
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

        df = new DecimalFormat("0.00");
        df_prec = new DecimalFormat("0.0000");

        storeClaw();
        stopAllMotors();
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
        for (OpticalDistanceSensorName ods : OpticalDistanceSensorName.values()) {
            setOpticalDistanceSensorLedEnable(ods, false);
        }
    }

    // Format for displaying decimals.
    public DecimalFormat df;
    public DecimalFormat df_prec;

}

