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

    /**
     * Drive to position. Simple calculation, drives in single rotational arc.
     * In general, not the most efficient path.
     * @param targetPosition
     * @param rate
     * @return boolean, true if currentPosition is near targetPosition.
     */
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

    public boolean rotateThenDriveToPosition(MecanumNavigation.Navigation2D targetPosition, double rate) {
        double distanceThresholdInches = 1;
        double angleThresholdRadians = 2.0 * (Math.PI/180.0);
        rate = Range.clip(rate,0,1);
        MecanumNavigation.Navigation2D currentPosition =
                (MecanumNavigation.Navigation2D)mecanumNavigation.currentPosition.clone();
        MecanumNavigation.Navigation2D deltaPosition = targetPosition.minusEquals(currentPosition);
        double deltaDistance = Math.sqrt( Math.pow(deltaPosition.x,2) + Math.pow(deltaPosition.y,2));

        double rateScale;
        // Not Close enough to target, keep moving
        if ( Math.abs(deltaPosition.theta) > angleThresholdRadians) {

            MecanumNavigation.Navigation2D rotationTarget = (MecanumNavigation.Navigation2D)currentPosition.clone();
            rotationTarget.theta = targetPosition.theta; // Only rotate to the target at first.
            Mecanum.Wheels wheels = mecanumNavigation.deltaWheelsFromPosition(rotationTarget);
            rateScale = rampDown(Math.abs(deltaPosition.theta)*(180/Math.PI), 50, 0.8, 0.1);
            wheels = wheels.scaleWheelPower(rateScale * rate);
            opMode.setDriveForMecanumWheels(wheels);
            return false;
            // After rotating, begin correcting translation.
        } else if (Math.abs(deltaPosition.x) > distanceThresholdInches ||
                   Math.abs(deltaPosition.y) > distanceThresholdInches) {
            Mecanum.Wheels wheels = mecanumNavigation.deltaWheelsFromPosition(targetPosition);
            rateScale = rampDown(deltaDistance, 5, 0.8, 0.1);
            wheels = wheels.scaleWheelPower(rateScale * rate);
            opMode.setDriveForMecanumWheels(wheels);
            return false;
        } else {  // Close enough
            opMode.setDriveForMecanumWheels(new Mecanum.Wheels(0,0,0,0));
            return true;
        }

    }

    public double rampDown(double errSignal, double signalRampDownThreshold, double maxRatio, double minRatio) {
        double output;
        double standardOutput = 1.0;
        if (errSignal > signalRampDownThreshold) {
            output = standardOutput;
        } else {
            output = standardOutput * (minRatio + (maxRatio - minRatio) * errSignal/signalRampDownThreshold);
        }
        return Range.clip(Math.abs(output),0,1);
    }


}
