package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

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
    private MecanumNavigation mecanumNavigation;
    private boolean use_telemetry           = true;
    private boolean forward_drive           = true;
    private boolean exponential_input       = false;
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
        // Update mecanum encoder navigation
        mecanumNavigation.update(new MecanumNavigation.WheelTicks(
                        getEncoderValue(MotorName.DRIVE_FRONT_LEFT),
                        getEncoderValue(MotorName.DRIVE_FRONT_RIGHT),
                        getEncoderValue(MotorName.DRIVE_BACK_LEFT),
                        getEncoderValue(MotorName.DRIVE_BACK_RIGHT)));
              mecanumNavigation.displayPosition();


        // Chord Commands
        if(controller.leftBumper()) {

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
            //telemetry.addData("Arm Encoder", getEncoderValue(MotorName.ARM_MOTOR));
            for (MotorName m : MotorName.values()) {
                telemetry.addData(m.name(), getEncoderValue(m));
            }
            telemetry.addData("Exponential", exponential_input);
            telemetry.addData("Slow", slow_mode);
        }
    }

    private void robotControls() {

        //Drive Motor control
        forward_drive = !controller.left_stick_buttonOnce();

        // Slow mode controls
        if (controller.dpadLeftOnce())
        {
            slow_mode = !slow_mode;
        }
        double fast_scale = 1.0;
        if (slow_mode)
        {
            fast_scale = 0.1;
        }


        double sign, exponential, max_rate;
        sign = forward_drive ? 1 : -1;
        //max_rate = slow_mode ? 1 : 0.5;
        exponential = exponential_input ? 3 : 1;
        setDriveForSimpleMecanum(
                sign * fast_scale * Math.pow(gamepad1.left_stick_x, exponential),
                sign *fast_scale * Math.pow(gamepad1.left_stick_y,exponential),
                Math.pow(gamepad1.right_stick_x,exponential),
                Math.pow(gamepad1.right_stick_y,exponential));


        // Arm controls
        if (analog_arm_control)
        {
            // Arm Control Analog
            double left_trigger = gamepad1.left_trigger; // Arm Dowm
            double right_trigger = gamepad1.right_trigger; // Arm Up

            if (left_trigger > 0.1) {
                setPower(MotorName.ARM_MOTOR, -(0.05 + 0.45*left_trigger));
            } else if (right_trigger > 0.1) {
                setPower(MotorName.ARM_MOTOR, (0.05 + 0.45*right_trigger));
            } else {
                setPower(MotorName.ARM_MOTOR, 0);
            }
        } else {
            // Arm Control Buttons
            if (gamepad1.dpad_up) {
                setPower(MotorName.ARM_MOTOR, Constants.RAISE_ARM_SPEED);
            } else if (gamepad1.dpad_down) {
                setPower(MotorName.ARM_MOTOR, Constants.LOWER_ARM_SPEED);
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
