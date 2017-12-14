package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Utilities.Color;
import org.firstinspires.ftc.teamcode.Utilities.Constants;
import org.firstinspires.ftc.teamcode.Utilities.Controller;
import org.firstinspires.ftc.teamcode.Utilities.Mecanum;
import org.firstinspires.ftc.teamcode.Utilities.MecanumNavigation;

/**
 * Created by ryderswan on 12/5/17.
 */


@TeleOp (name="Manual Mecanum", group="Manual")
//@Disabled
public class Manual extends RobotHardware {
    private Controller controller = null;
    public MecanumNavigation mecanumNavigation;
    public double averagePeriodSeconds = 0;
    private boolean use_telemetry           = true;
    private boolean forward_drive           = true;
    private boolean exponential_input       = true;
    private boolean analog_arm_control      = false;
    private boolean slow_mode               = false;

    @Override
    public void init() {
        super.init();
        controller = new Controller(gamepad1);
        mecanumNavigation = new MecanumNavigation(this,
                new MecanumNavigation.DriveTrainMecanum(
                        Constants.WHEELBASE_LENGTH_IN, Constants.WHEELBASE_WIDTH_IN,
                        Constants.DRIVE_WHEEL_DIAMETER_INCHES, Constants.DRIVE_WHEEL_STEPS_PER_ROT));
        mecanumNavigation.initialize(new MecanumNavigation.Navigation2D(0,0,0),
                new MecanumNavigation.WheelTicks(getEncoderValue(MotorName.DRIVE_FRONT_LEFT),
                        getEncoderValue(MotorName.DRIVE_FRONT_RIGHT),
                        getEncoderValue(MotorName.DRIVE_BACK_LEFT),
                        getEncoderValue(MotorName.DRIVE_BACK_RIGHT)));
    }

    @Override
    public void loop() {
        // Keep controller rising edge trigger updated.
        controller.update();
        // Average length of an execution period
        averagePeriodSeconds = periodSec();
        // Update mecanum encoder navigation via opMode context.
        mecanumNavigation.update();


        // Chord Commands
        if(controller.leftBumper()) {

            stopAllMotors(); // Safety first! (Otherwise motors can run uncontrolled.)

            // Reset navigation position to zero.
            if (controller.YOnce()) {
                mecanumNavigation.setCurrentPosition(new MecanumNavigation.Navigation2D(0, 0, 0));
            }
            // Toggle analog arm control.
            if (controller.XOnce()) {
                analog_arm_control = ! analog_arm_control;
            }
            // Toggle exponential input
            if (controller.BOnce()) {
                exponential_input = ! exponential_input;
            }
            // Cage claw
            if (controller.AOnce()) {
                storeClaw();
            }

        } else {
            // Non-chord robot inputs
            robotControls();
        }


        if (use_telemetry)
        {
            telemetry.addData("Exponential", exponential_input);
            telemetry.addData("Slow", slow_mode);
            telemetry.addData("Forward Drive", forward_drive);
            telemetry.addData("Arm Encoder", getEncoderValue(MotorName.ARM_MOTOR));
            telemetry.addLine(); // Visual Space
            mecanumNavigation.displayPosition();
            telemetry.addLine(); // Visual Space
        }
    }

    private void robotControls() {

        //Drive Motor control
        if (controller.leftStickButtonOnce()) {
            forward_drive = !forward_drive;
        }

        // Slow mode controls
        if (controller.rightStickButtonOnce()) {
            slow_mode = !slow_mode;
        }

        double sign, exponential, max_rate;
        sign = forward_drive ? 1 : -1;
        max_rate = slow_mode ? 0.3 : 1;
        exponential = exponential_input ? 3 : 1;
        setDriveForSimpleMecanum(
                sign * max_rate * Math.pow(gamepad1.left_stick_x, exponential),
                sign * max_rate * Math.pow(gamepad1.left_stick_y, exponential),
                max_rate * Math.pow(gamepad1.right_stick_x, exponential),
                max_rate * Math.pow(gamepad1.right_stick_y, exponential));


        // Arm controls
        if (analog_arm_control)
        {
            // Arm Control Analog
            double left_trigger = gamepad1.left_trigger; // Arm Dowm
            double right_trigger = gamepad1.right_trigger; // Arm Up

            if (left_trigger > 0.1) {
                setPower(MotorName.ARM_MOTOR, sign * -(0.05 + 0.45*left_trigger));
            } else if (right_trigger > 0.1) {
                setPower(MotorName.ARM_MOTOR, sign * (0.05 + 0.45*right_trigger));
            } else {
                setPower(MotorName.ARM_MOTOR, 0);
            }
        } else {
            // Arm Control Buttons
            if (gamepad1.dpad_up) {
                setPower(MotorName.ARM_MOTOR, sign * Constants.RAISE_ARM_SPEED);
            } else if (gamepad1.dpad_down) {
                setPower(MotorName.ARM_MOTOR, sign * Constants.LOWER_ARM_SPEED);
            } else {
                setPower(MotorName.ARM_MOTOR, 0.0);
            }
        }

        //Claw Control
        if (gamepad1.x) {
            openClaw();
        } else if (gamepad1.y) {
            closeClaw();
        } else if (gamepad1.b) {
            slightOpenClaw();
        } else if (gamepad1.a && !analog_arm_control) {
            setPositionClaw(gamepad1.right_trigger);
        }


    }


}
