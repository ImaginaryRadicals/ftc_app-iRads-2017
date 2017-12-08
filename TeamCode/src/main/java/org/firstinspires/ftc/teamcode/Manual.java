package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Utilities.Constants;
import org.firstinspires.ftc.teamcode.Utilities.Mecanum;

/**
 * Created by ryderswan on 12/5/17.
 */


@TeleOp (name="Manual Mecanum", group="Manual")
//@Disabled
public class Manual extends RobotHardware {
    public boolean use_telemetry = true;
    public boolean forward_drive = true;

    @Override
    public void loop() {

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
        if (gamepad1.start)
        {
            // Arm Control Analog
            double left_trigger = gamepad1.left_trigger; // Arm Dowm
            double right_trigger = gamepad1.right_trigger; // Arm Up

            if (left_trigger > 0.1) {
                setPower(MotorName.ARM_MOTOR, -(0.1 + 0.9*left_trigger));
            } else if (right_trigger > 0.1) {
                setPower(MotorName.ARM_MOTOR, (0.1 + 0.9*right_trigger));
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
