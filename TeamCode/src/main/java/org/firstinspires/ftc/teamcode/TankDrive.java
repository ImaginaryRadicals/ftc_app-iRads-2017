package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Utilities.Controller;
import org.firstinspires.ftc.teamcode.Utilities.Mecanum;

/**
 *  Simple tank drive mode.
 */


@TeleOp (name="Manual TankDrive", group="Manual")
//@Disabled
public class TankDrive extends RobotHardware {

    @Override
    public void loop() {

        //Drive Motor control
        setDriveForTank(-gamepad1.left_stick_y, -gamepad1.right_stick_y);


        // Arm Control
        if (gamepad1.dpad_up) {
            setPower(MotorName.ARM_MOTOR, .2);
        }else if (gamepad1.dpad_down) {
            setPower(MotorName.ARM_MOTOR, -.2);
        } else {
            setPower(MotorName.ARM_MOTOR, 0);
        }

        //Claw Control
        if (gamepad1.x) {
            openClaw();
        } else if (gamepad1.y) {
            closeClaw();
        } else if (gamepad1.b) {
            slightOpenClaw();
        } else if (gamepad1.a) {
            setPositionClaw(gamepad1.left_trigger);
        }


    }


}
