package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.Utilities.Mecanum;

/**
 * Created by ryderswan on 12/5/17.
 */

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




    }


}
