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

    Controller controller = new Controller(gamepad1);

    @Override
    public void loop() {

        // Keep button one shot triggers updated.
        controller.update();

        //Drive Motor control
        setDriveForTank(-controller.left_stick_y, -controller.right_stick_y);


        // Arm Control
        if (controller.dpadUp()) {
            setPower(MotorName.ARM_MOTOR, 1);
        }else if (controller.dpadDown()) {
            setPower(MotorName.ARM_MOTOR, -1);
        } else {
            setPower(MotorName.ARM_MOTOR, 0);
        }

        //Claw Control
        if (controller.X()) {
            openClaw();
        } else if (controller.Y()) {
            closeClaw();
        } else if (controller.B()) {
            slightOpenClaw();
        }


    }


}
