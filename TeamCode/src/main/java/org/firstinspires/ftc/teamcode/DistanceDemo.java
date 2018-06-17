package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

@TeleOp(name = "DistanceDemo")
public class DistanceDemo extends OpMode {

    OpticalDistanceSensor distance;

    @Override
    public void init() {
        distance = hardwareMap.opticalDistanceSensor.get("distance");
    }

    @Override
    public void loop() {
        telemetry.addData("Distance", distance.getRawLightDetected());

    }
}
