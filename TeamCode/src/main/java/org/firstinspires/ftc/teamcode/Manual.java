package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

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
            raiseArm();
        }else if (gamepad1.dpad_down) {
            lowerArm();
        }

        //Claw Control
        if (gamepad1.x) {
            openClaw();
        } else if (gamepad1.y) {
            closeClaw();
        } else if (gamepad1.b) {
            slightOpenClaw();
        }


//                  BTW RYDER WAS HERE!!!!!


    }


}
