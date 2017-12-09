package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Utilities.Constants;
import org.firstinspires.ftc.teamcode.Utilities.Controller;
import org.firstinspires.ftc.teamcode.Utilities.Mecanum;

/**
 * Created by ryderswan on 12/5/17.
 */


@TeleOp (name="Manual Mecanum", group="Manual")
//@Disabled
public class Manual extends RobotHardware {
    private Controller controller = null;
    private boolean use_telemetry = true;
    private boolean forward_drive = true;
    private boolean is_analog_arm_control = false;

    @Override
    public void init() {
        super.init();
        controller = new Controller(gamepad1);
    }

    @Override
    public void loop() {
        // Keep controller rising edge trigger updated.
        controller.update();

        //Drive Motor control
        forward_drive = !gamepad1.right_bumper;
        if(forward_drive) {
            setDriveForSimpleMecanum(
                    gamepad1.left_stick_x, gamepad1.left_stick_y,
                    gamepad1.right_stick_x, gamepad1.right_stick_y);
        }
        else{
            setDriveForSimpleMecanum(
                    -gamepad1.left_stick_x, -gamepad1.left_stick_y,
                    gamepad1.right_stick_x, gamepad1.right_stick_y);
        }

        // Toggle analog arm control.
        if (controller.leftBumperOnce()) {
            is_analog_arm_control = ! is_analog_arm_control;
        }
        if (is_analog_arm_control)
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
        }

        if (use_telemetry)
        {
            telemetry.addData("Arm Encoder", getEncoderValue(MotorName.ARM_MOTOR));
        }
    }


}
