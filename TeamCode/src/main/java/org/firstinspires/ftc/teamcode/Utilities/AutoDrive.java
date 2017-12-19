package org.firstinspires.ftc.teamcode.Utilities;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.RobotHardware;

/**
 * Created by Ashley on 12/18/2017.
 */

public class AutoDrive {

    private RobotHardware opMode;
    private MecanumNavigation mecanumNavigation;


    public AutoDrive(RobotHardware opMode, MecanumNavigation mecanumNavigation) {
        this.opMode = opMode;
        this.mecanumNavigation = mecanumNavigation;
    }

    public boolean driveToPosition(MecanumNavigation.Navigation2D targetPosition, double rate) {
        double distanceThresholdInches = 1;
        double angleThresholdRadians = 2 * (2*Math.PI/180);
        rate = Range.clip(rate,0,1);
        MecanumNavigation.Navigation2D currentPosition =
                (MecanumNavigation.Navigation2D)mecanumNavigation.currentPosition.clone();
        MecanumNavigation.Navigation2D deltaPosition = targetPosition.minusEquals(currentPosition);

        // Not Close enough to target, keep moving
        if ( Math.abs(deltaPosition.x) > distanceThresholdInches ||
                Math.abs(deltaPosition.y) > distanceThresholdInches ||
                Math.abs(deltaPosition.theta) > angleThresholdRadians) {

            Mecanum.Wheels wheels = mecanumNavigation.deltaWheelsFromPosition(targetPosition);
            wheels.scaleWheelPower(rate);
            opMode.setDriveForMecanumWheels(wheels);
            return false;
        } else {  // Close enough
            opMode.setDriveForMecanumWheels(new Mecanum.Wheels(0,0,0,0));
            return true;
        }

    }

}
