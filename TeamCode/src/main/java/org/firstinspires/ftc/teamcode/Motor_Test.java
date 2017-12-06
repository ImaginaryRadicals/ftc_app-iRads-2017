package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Utilities.Mecanum;

/**
 * Created by ryderswan on 12/5/17.
 */


@TeleOp (name="Motor Test", group="Manual")
//@Disabled
public class Motor_Test  extends RobotHardware {
    @Override

    public void loop() {
        if (gamepad1.left_trigger > 0.1) {
            setPower(MotorName.ARM_MOTOR, gamepad1.left_trigger);
        } else if (gamepad1.right_trigger > 0.1) {
            setPower(MotorName.ARM_MOTOR, gamepad1.right_trigger);
        }



    }


}
