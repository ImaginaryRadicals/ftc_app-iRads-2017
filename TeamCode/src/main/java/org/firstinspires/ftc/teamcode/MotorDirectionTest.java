package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 *  Motor Direction test extending basic Tank Drive.
 */


@TeleOp (name="Tank Motor Dir Test", group="Diagnostic")

public class MotorDirectionTest extends TankDrive {


    @Override
    public void loop() {

        if (gamepad1.start) {
            if (gamepad1.right_bumper) {
                setPower(MotorName.DRIVE_FRONT_RIGHT, 1);
            } else {
                setPower(MotorName.DRIVE_FRONT_RIGHT, 0);
            }

            if (gamepad1.right_trigger > 0.2) {
                setPower(MotorName.DRIVE_BACK_RIGHT, 1);
            } else {
                setPower(MotorName.DRIVE_BACK_RIGHT, 0);
            }

            if (gamepad1.left_bumper) {
                setPower(MotorName.DRIVE_FRONT_LEFT, 1);
            } else {
                setPower(MotorName.DRIVE_FRONT_LEFT, 0);
            }

            if (gamepad1.left_trigger > 0.2) {
                setPower(MotorName.DRIVE_BACK_LEFT, 1);
            } else {
                setPower(MotorName.DRIVE_BACK_LEFT, 0);
            }

        } else {

            // Run original code if start button isn't held.
            super.loop();

        }

    }


}
