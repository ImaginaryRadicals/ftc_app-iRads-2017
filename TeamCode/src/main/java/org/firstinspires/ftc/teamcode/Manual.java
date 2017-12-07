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
    @Override

    public void loop() {

        //Drive Motor control
        setDriveForMecanumForSpeed(Mecanum.joystickToMotion(
                gamepad1.left_stick_x, gamepad1.left_stick_y,
                gamepad1.right_stick_x, gamepad1.right_stick_y));



        // Arm Control
        if (gamepad1.dpad_up) {
            setPower(MotorName.ARM_MOTOR, Constants.RAISE_ARM_SPEED);
        }else if (gamepad1.dpad_down) {
            setPower(MotorName.ARM_MOTOR, Constants.LOWER_ARM_SPEED);
        }else {
            setPower(MotorName.ARM_MOTOR, 0.0);
        }

        //Claw Control
        if (gamepad1.x) {
            openClaw();
        } else if (gamepad1.y) {
            closeClaw();
        } else if (gamepad1.b) {
            slightOpenClaw();
        }


    }


}
