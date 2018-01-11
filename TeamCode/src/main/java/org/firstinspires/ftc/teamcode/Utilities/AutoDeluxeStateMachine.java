package org.firstinspires.ftc.teamcode.Utilities;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.teamcode.Auto;
import org.firstinspires.ftc.teamcode.AutoDeluxe;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.Vision.SimpleVuforia;

import java.util.ArrayList;
import java.util.Arrays;

/**
 * Created by Ashley on 12/16/2017.
 */

public class AutoDeluxeStateMachine {

    public enum AutoState {
        STATE_START,
        STATE_LOWER_ARM,
        STATE_DETECT_COLOR,
        STATE_HIT_DETECTED_JEWEL_CW,
        STATE_HIT_OTHER_JEWEL_CCW,
        STATE_ABORT_JEWEL,
        STATE_DRIVE_TO_GLYPH_BOX,
        STATE_LOWER_GLYPH_ARM,
        STATE_STOP,
    }

    public AutoState state = AutoState.STATE_START;
    private AutoDeluxe opMode;
    private ElapsedTime stateLoopTimer = new ElapsedTime();
    private double lastStateLoopPeriod = 0;
    private ElapsedTime stateTimer = new ElapsedTime();

    // Colors
    public Color.Ftc teamColor = Color.Ftc.UNKNOWN;
    public Color.Ftc jewelColor = Color.Ftc.UNKNOWN;

    // Kinematics
    private double timeHitJewel = 0.5;
    private double powerHitJewel = 0.2;
    public RobotHardware.StartPosition startPosition;
    private int initialArmEncoderTicks = 0;
    private int currentDriveWaypoint = 0;

    // Motions for each starting position
    private ArrayList<MecanumNavigation.Navigation2D> blueCornerWaypoints = new ArrayList<>(Arrays.asList(
            new MecanumNavigation.Navigation2D(36,0, degreesToRadians(0)),
            new MecanumNavigation.Navigation2D(36,12, degreesToRadians(0))));
    private ArrayList<MecanumNavigation.Navigation2D> blueCenterWaypoints = new ArrayList<>(Arrays.asList(
            new MecanumNavigation.Navigation2D(36,0, degreesToRadians(0)),
            new MecanumNavigation.Navigation2D(36,12, degreesToRadians(90))));
    private ArrayList<MecanumNavigation.Navigation2D> redCornerWaypoints = new ArrayList<>(Arrays.asList(
            new MecanumNavigation.Navigation2D(-36,0, degreesToRadians(0)),
            new MecanumNavigation.Navigation2D(-36,-12, degreesToRadians(180))));
//    private ArrayList<MecanumNavigation.Navigation2D> redCenterWaypoints = new ArrayList<>(Arrays.asList(
//            new MecanumNavigation.Navigation2D(-36,0,degreesToRadians(0)),
//            new MecanumNavigation.Navigation2D(-36,12,degreesToRadians(90))));
    private ArrayList<MecanumNavigation.Navigation2D> redCenterWaypoints = new ArrayList<>(Arrays.asList(
            new MecanumNavigation.Navigation2D(12,0, degreesToRadians(0)),
            new MecanumNavigation.Navigation2D(12,12, degreesToRadians(90)),
            new MecanumNavigation.Navigation2D(0,12, degreesToRadians(180)),
            new MecanumNavigation.Navigation2D(0,0, degreesToRadians(270)),
            new MecanumNavigation.Navigation2D(0,0, degreesToRadians(360))));


    public AutoDeluxeStateMachine(AutoDeluxe opMode, Color.Ftc teamColor, RobotHardware.StartPosition startPosition) {
        this.opMode = opMode;
        this.teamColor = teamColor;
        this.startPosition = startPosition;
    }


    public void init() {
        stateLoopTimer.reset();
        stateTimer.reset();
    }


    public void update() {

        lastStateLoopPeriod = stateLoopTimer.seconds();
        stateLoopTimer.reset();

        if (state == AutoState.STATE_START) {
            initialArmEncoderTicks = opMode.getEncoderValue(RobotHardware.MotorName.ARM_MOTOR);
            stateTimer.reset();
            state = AutoState.STATE_LOWER_ARM;
        } else if (state == AutoState.STATE_LOWER_ARM) {
            opMode.closeClaw();
            opMode.moveServoAtRate(RobotHardware.ServoName.JEWEL_ARM, Constants.JEWEL_ARM_BOTTOM,0.7);

            // Wait 1 second before lifting arm.
            if(stateTimer.seconds() > 1) {
                if (opMode.getEncoderValue(RobotHardware.MotorName.ARM_MOTOR) < initialArmEncoderTicks + 500) {
                    opMode.setPower(RobotHardware.MotorName.ARM_MOTOR, 0.5);
                } else {
                    opMode.setPower(RobotHardware.MotorName.ARM_MOTOR, 0);
                }
            }
            // Give jewel arm 2.5 seconds to settle, then detect the color.
            if(stateTimer.seconds() > 2) {
                state = AutoState.STATE_DETECT_COLOR;
                stateTimer.reset();
            }
        } else if (state == AutoState.STATE_DETECT_COLOR) {
            // Repeatedly attempt to capture a non-unknown color value.
            setJewelColor( opMode.getJewelColor() );

            // If color is found or 2 second timeout elapses, go to next state
            if(jewelColor != Color.Ftc.UNKNOWN || stateTimer.seconds() > 2) {
                stateTimer.reset();
                if (jewelColor == teamColor) {
                    state = AutoState.STATE_HIT_OTHER_JEWEL_CCW;
                } else if (jewelColor == Color.Ftc.UNKNOWN) {
                    state = AutoState.STATE_ABORT_JEWEL;
                } else {
                    state = AutoState.STATE_HIT_DETECTED_JEWEL_CW;
                }
            }
        } else if (state == AutoState.STATE_HIT_DETECTED_JEWEL_CW) {
            if(stateTimer.seconds() < timeHitJewel) {
                opMode.setDriveForTank(powerHitJewel,-powerHitJewel);
            } else if (stateTimer.seconds() < 2*timeHitJewel ) {
                opMode.armServoTop();
                opMode.setDriveForTank(-powerHitJewel,powerHitJewel);
            } else {
                opMode.stopAllMotors();
                stateTimer.reset();
                state = AutoState.STATE_DRIVE_TO_GLYPH_BOX;
            }
        } else if (state == AutoState.STATE_HIT_OTHER_JEWEL_CCW) {
            if(stateTimer.seconds() < timeHitJewel) {
                opMode.setDriveForTank(-powerHitJewel,powerHitJewel);
            } else if (stateTimer.seconds() < 2*timeHitJewel ) {
                opMode.armServoTop();
                opMode.setDriveForTank(powerHitJewel,-powerHitJewel);
            } else {
                opMode.stopAllMotors();
                stateTimer.reset();
                state = AutoState.STATE_DRIVE_TO_GLYPH_BOX;
            }

        } else if (state == AutoState.STATE_ABORT_JEWEL) {
            opMode.armServoTop();
            if (stateTimer.seconds() > 1) {
                state = AutoState.STATE_DRIVE_TO_GLYPH_BOX;
                stateTimer.reset();
            }
        } else if (state == AutoState.STATE_DRIVE_TO_GLYPH_BOX) {
            boolean arrivedAtWaypoint = false;
            ArrayList<MecanumNavigation.Navigation2D> waypointArray;
            // Added to ensure array is initialized.
            waypointArray = new ArrayList<>(Arrays.asList(new MecanumNavigation.Navigation2D(0,0,0)));

            // Select Driving waypoints
            if (teamColor == Color.Ftc.BLUE) {
                if (startPosition == RobotHardware.StartPosition.FIELD_CORNER) {
                    waypointArray = blueCornerWaypoints;
                } else if (startPosition == RobotHardware.StartPosition.FIELD_CENTER) {
                    waypointArray = blueCenterWaypoints;
                }
            } else if (teamColor == Color.Ftc.RED) {
                if (startPosition == RobotHardware.StartPosition.FIELD_CORNER) {
                    waypointArray = redCornerWaypoints;
                } else if (startPosition == RobotHardware.StartPosition.FIELD_CENTER) {
                    waypointArray = redCenterWaypoints;
                }
            } else {
                opMode.stopAllMotors();
                stateTimer.reset();
                state = AutoState.STATE_STOP;
            }

            // Do the driving
            if(currentDriveWaypoint < waypointArray.size() ) {
                // Show Target Status and debug info
                opMode.telemetry.addData("Current Waypoint: ", currentDriveWaypoint);
                opMode.telemetry.addData("Target", waypointArray.get(currentDriveWaypoint).toString());
                arrivedAtWaypoint = opMode.autoDrive.rotateThenDriveToPosition(waypointArray.get(currentDriveWaypoint),0.5);
            }
            if ( arrivedAtWaypoint) {
                ++currentDriveWaypoint;
            }



            // Next State Logic
            if (currentDriveWaypoint >= waypointArray.size()) {
                opMode.stopAllMotors();
                stateTimer.reset();
                state = AutoState.STATE_LOWER_GLYPH_ARM;
            }
        } else if (state == AutoState.STATE_LOWER_GLYPH_ARM) {
            if ( opMode.getEncoderValue(RobotHardware.MotorName.ARM_MOTOR) > initialArmEncoderTicks) {
                opMode.setPower(RobotHardware.MotorName.ARM_MOTOR,-0.2);
            } else {
                opMode.setPower(RobotHardware.MotorName.ARM_MOTOR, 0);
                opMode.stopAllMotors();
                stateTimer.reset();
                state = AutoState.STATE_STOP;
            }
        } else if (state == AutoState.STATE_STOP) {
            opMode.stopAllMotors();
            opMode.stop();
        } else {
            // error
            opMode.stopAllMotors();
        }
    }



    private void setJewelColor(Color.Ftc detectedJewelColor) {
        if ( detectedJewelColor != Color.Ftc.UNKNOWN) {
            this.jewelColor = detectedJewelColor;
        }
    }

    private double degreesToRadians(double degrees) {
        return degrees * Math.PI / 180;
    }

    private double radiansToDegrees(double radians) {
        return radians * 180 / Math.PI;
    }




}