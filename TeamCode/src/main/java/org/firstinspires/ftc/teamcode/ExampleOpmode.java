package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "example TeleOp")
public class ExampleOpmode extends OpMode{

    DcMotor motor;

    @Override
    public void init() {
        //Run ONCE when you press INIT

        left.setPower = hardwareMap.dcMotor.get("left");
        right.setPower = hardwareMap.dcMotor.get("right");
    }

    @Override
    public void init_loop() {
        super.init_loop();
        //Run OVER AND OVER between INIT and START
    }

    @Override
    public void start() {
        super.start();
        //Run ONCE when you press START
    }

    @Override
    public void loop() {
        //Run OVER AND OVER while the program is running
        left.setPower = (gamepad1.left_stick_y);
        right.setPower(gamepad1.right_stick_y);
    }

    @Override
    public void stop() {
        super.stop();
        //Run ONCE when you press STOP
    }
}

