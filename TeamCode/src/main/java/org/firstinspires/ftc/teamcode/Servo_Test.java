/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


@TeleOp(name="Servo Test", group="Test")

public class Servo_Test extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    //private DcMotor armMotor = null;
    private Servo leftServo = null;
    private Servo rightServo =null;

    private double servoLeftPos = 0;
    private double servoRightPos = 0;
    private double servoRate = 5;
    private ElapsedTime servoTimer = new ElapsedTime();


    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
/*
       armMotor  = hardwareMap.get(DcMotor.class, "armMotor");
       armMotor.setDirection(DcMotor.Direction.FORWARD);
       armMotor.setPower(0);
       armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
*/
       leftServo = hardwareMap.get(Servo.class, "leftServo");
       leftServo.setPosition(0);
       rightServo = hardwareMap.get(Servo.class, "rightServo");
       rightServo.setPosition(0);

       telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
        servoTimer.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */

    double stepSize, stepTime;

    @Override
    public void loop() {

        stepTime = servoTimer.seconds();
        servoTimer.reset();

        stepSize = servoRate * stepTime * gamepad1.left_stick_x;
        servoLeftPos = Range.clip(stepSize + servoLeftPos, 0, 1);
        leftServo.setPosition(servoLeftPos);
        stepSize = servoRate * stepTime * gamepad1.right_stick_x;
        servoRightPos = Range.clip(stepSize + servoRightPos, 0, 1);
        rightServo.setPosition(servoRightPos);

/*
        if (gamepad1.y)
            armMotor.setPower(1);
        else if (gamepad1.a)
            armMotor.setPower(-1);
        else
            armMotor.setPower(0);
*/
        telemetry.addData("leftServoPos", servoLeftPos);
        telemetry.addData("rightServoPos", servoRightPos);
    }


    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
