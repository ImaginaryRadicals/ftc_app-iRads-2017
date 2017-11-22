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

import android.content.Context;
import android.text.ParcelableSpan;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import java.io.File;
import java.io.FileOutputStream;
import java.io.OutputStreamWriter;
import java.io.Writer;

/**
    The only purpose of this opmode is to demonstrate recording data to a file.
    The hope is to record a stream of ascii data which can easily be displayed
    on a pc using python.
 */

@TeleOp(name="File Recorder Demo", group="Test")

public class FileRecord_OpMode extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime loopTimer = new ElapsedTime();
    private FileOutputStream outputStream;

    // Hardware
    OpticalDistanceSensor odsSensor;



    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {

        try {
            odsSensor = hardwareMap.get(OpticalDistanceSensor.class, "odsSensor");
        } catch (Exception e ) {
            telemetry.addData("Error:", "Unable to find odsSensor");
        }

        try {
            telemetry.addData("File Location", hardwareMap.appContext.getFilesDir());
            outputStream = hardwareMap.appContext.openFileOutput("fileTest", Context.MODE_PRIVATE);
        } catch (Exception e) {
            e.printStackTrace();
            requestOpModeStop();
        }

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
        loopTimer.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    Double loopInterval = new Double(0);
    @Override
    public void loop() {

        loopInterval = loopTimer.seconds();
        loopTimer.reset();

        try {
            recordODSDiagnostic("ODSLevel", runtime.seconds(), 0, odsSensor.getRawLightDetected(),
                                odsSensor.getLightDetected(), outputStream);
        } catch (Exception e) {
            telemetry.addData("Cannot read from ODS sensor", "");
            recordODSDiagnostic("ODSLevel", runtime.seconds(), 0, 0,0, outputStream);
        }

        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Loop Time: ", loopInterval.toString());
        //telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {

        try {
            outputStream.close();
            telemetry.addData("File Location", hardwareMap.appContext.getFilesDir());
        } catch (Exception e) {
            e.printStackTrace();
        }
    }


    /*
     *  Text recorder function
     */
    private void recordText(String string, FileOutputStream outputStream){

        try {
            outputStream.write(string.getBytes());
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    private void recordODSDiagnostic(String name, double time, double distance,
                                     double valueRaw, double valueScaled,
                                     FileOutputStream outputStream) {
        recordText("name: " + name + ", ", outputStream);
        recordText("time: " + Double.toString(time) + ", ", outputStream);
        recordText("distance: " + Double.toString(distance) + ", ", outputStream);
        recordText("valueRaw: " + Double.toString(valueRaw) + ", ", outputStream);
        recordText("valueScaled: " + Double.toString(valueScaled) + ", ", outputStream);
        recordText(",," + System.getProperty("line.separator") , outputStream);

    }
}
