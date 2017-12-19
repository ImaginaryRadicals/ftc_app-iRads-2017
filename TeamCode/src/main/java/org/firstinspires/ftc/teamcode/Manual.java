package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Utilities.AutoDrive;
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

    public Controller controller = null;
    public MecanumNavigation mecanumNavigation;
    public AutoDrive autoDrive;
    private boolean use_telemetry = true;
    private boolean forward_drive = true;
    private boolean exponential_input = true;
    private boolean analog_arm_control = true;
    private boolean slow_mode = false;
    private ClawState clawState = ClawState.CLAW_STOWED;
    private boolean fastMode = false;

    // Variables for the claw states.
    private enum ClawState {
        CLAW_STOWED, CLAW_OPEN, CLAW_RELEASE, CLAW_CLOSED, CLAW_TESTING,
    }


    @Override
    public void init() {
        super.init();
        controller = new Controller(gamepad1);
        mecanumNavigation = new MecanumNavigation(this,
                new MecanumNavigation.DriveTrainMecanum(
                        Constants.WHEELBASE_LENGTH_IN, Constants.WHEELBASE_WIDTH_IN,
                        Constants.DRIVE_WHEEL_DIAMETER_INCHES, Constants.DRIVE_WHEEL_STEPS_PER_ROT));
        mecanumNavigation.initialize(new MecanumNavigation.Navigation2D(0, 0, 0),
                new MecanumNavigation.WheelTicks(getEncoderValue(MotorName.DRIVE_FRONT_LEFT),
                        getEncoderValue(MotorName.DRIVE_FRONT_RIGHT),
                        getEncoderValue(MotorName.DRIVE_BACK_LEFT),
                        getEncoderValue(MotorName.DRIVE_BACK_RIGHT)));
        autoDrive = new AutoDrive(this, mecanumNavigation);
    }

    @Override
    public void start() {
        super.start();
        storeClaw();
        armServoTop();
    }

    @Override
    public void loop() {
        // Keep timers updated
        super.loop();
        // Keep controller rising edge trigger updated.
        controller.update();
        // Update mecanum encoder navigation via opMode context.
        mecanumNavigation.update();


        // Chord Commands
        if (controller.leftBumper() && controller.rightBumper()) {

            if(!controller.B() && !controller.X()) {
                stopAllMotors(); // Safety first! (Otherwise motors can run uncontrolled.)
            }
            jewelArmTesting();
            jewelServoRateTesting();
            clawTestControls();

            // Reset navigation position to zero.
            if (controller.YOnce()) {
                mecanumNavigation.setCurrentPosition(new MecanumNavigation.Navigation2D(0, 0, 0));
            }
            // Toggle analog arm control.
            if (controller.X()) {
                autoDrive.rotateThenDriveToPosition(new MecanumNavigation.Navigation2D(0,0,0), 1);
            }
            // Toggle exponential input
            if (controller.B()) {  // Removed 'once' trigger.
                driveToPosition(mecanumNavigation, new MecanumNavigation.Navigation2D(0,0,0), 1);
            }
            // Full power mode
            if (controller.AOnce()) {
                fastMode = !fastMode;
            }
        } else {
            // Non-chord robot inputs
            robotControls();
        }


        if (use_telemetry) {
            telemetry.addData("Exponential", exponential_input);
            telemetry.addData("Slow", slow_mode);
            telemetry.addData("Fast", fastMode);
            telemetry.addData("Forward Drive", forward_drive);
            telemetry.addData("Arm Encoder", getEncoderValue(MotorName.ARM_MOTOR));
            telemetry.addData("Jewel Target Position", df.format(getAngle(ServoName.JEWEL_ARM)));
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

        double sign = forward_drive ? 1 : -1;
        double max_rate = 0.8;
        if (slow_mode) {
            max_rate = 0.3;
        } else if (fastMode) {
            max_rate = 1.0;
        }
        double exponential = exponential_input ? 3 : 1;
        setDriveForSimpleMecanum(
                sign * max_rate * Math.pow(gamepad1.left_stick_x, exponential),
                sign * max_rate * Math.pow(gamepad1.left_stick_y, exponential),
                max_rate * Math.pow(gamepad1.right_stick_x, exponential),
                max_rate * Math.pow(gamepad1.right_stick_y, exponential));


        // Arm controls
        if (analog_arm_control) {
            // Arm Control Analog
            double left_trigger = gamepad1.left_trigger; // Arm Dowm
            double right_trigger = gamepad1.right_trigger; // Arm Up

            if (left_trigger > 0.1) {
                setPower(MotorName.ARM_MOTOR, sign * -(0.05 + 0.45 * left_trigger));
            } else if (right_trigger > 0.1) {
                setPower(MotorName.ARM_MOTOR, sign * (0.05 + 0.45 * right_trigger));
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

        clawStateMachine();

    }



    // Claw Control
    // Right closes and Left Opens
    private void clawStateMachine() {
        if (clawState == ClawState.CLAW_STOWED) {
            storeClaw();
            if (controller.rightBumperOnce()) {
                clawState = ClawState.CLAW_OPEN;
            }
        } else if (clawState == ClawState.CLAW_OPEN) {
            openClaw();
            if (controller.rightBumperOnce()) {
                clawState = ClawState.CLAW_CLOSED;
            } else if (controller.leftBumperOnce()) {
                clawState = ClawState.CLAW_STOWED;
            }
        } else if (clawState == ClawState.CLAW_CLOSED) {
            closeClaw();
            if (controller.leftBumperOnce()) {
                clawState = ClawState.CLAW_RELEASE;
            }
        } else if (clawState == ClawState.CLAW_RELEASE) {
            slightOpenClaw();
            if (controller.rightBumperOnce()) {
                clawState = ClawState.CLAW_CLOSED;
            } else if (controller.leftBumperOnce()) {
                clawState = ClawState.CLAW_OPEN;
            }
        } else if (clawState == ClawState.CLAW_TESTING) {
            // No position commanded, leave where it is.
            if (controller.rightBumperOnce()) {
                clawState = ClawState.CLAW_CLOSED;
            } else if (controller.leftBumperOnce()) {
                clawState = ClawState.CLAW_OPEN;
            }
        }
    }

    private void clawTestControls() {
        double clawRate = 1;
        // If (and only if) the claw is moved in testing, set state to leave claw where it is.
        if ( controller.left_stick_y != 0 ) {
            clawState = ClawState.CLAW_TESTING;
        }
        double clawTarget = controller.left_stick_y * clawRate * getLastPeriodSec() + getAngle(ServoName.CLAW_LEFT);
        clawTarget = Range.clip(clawTarget, 0, 1);
        setPositionClaw(clawTarget);
    }

    private void jewelArmTesting() {
        double servoRate = 1;
        double stepSize = servoRate * getLastPeriodSec() * -controller.right_stick_y;
        double jewelServoTargetPosition = Range.clip(stepSize + getAngle(ServoName.JEWEL_ARM), 0, 1);
        setAngle(ServoName.JEWEL_ARM, jewelServoTargetPosition);
    }

    private void jewelServoRateTesting() {
        // Alternative Jewel Arm Control Demo
        if (analog_arm_control && controller.dpadDown()) {
            moveServoAtRate(ServoName.JEWEL_ARM, Constants.JEWEL_ARM_BOTTOM, 0.4);
        } else if (analog_arm_control && controller.dpadUp()) {
            moveServoAtRate(ServoName.JEWEL_ARM, Constants.JEWEL_ARM_INITIAL, 0.4);
        }
    }

}