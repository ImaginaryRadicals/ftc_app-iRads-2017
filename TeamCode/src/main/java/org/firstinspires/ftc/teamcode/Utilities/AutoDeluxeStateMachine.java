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
        STATE_PROCESS_VUMARK,
        STATE_DRIVE_TO_GLYPH_BOX,
        STATE_DISMOUNT,
        STATE_ALIGN_W_OFFSETS,
        STATE_APPROACH_GLYPH_BOX,
        STATE_ROTATE_AND_INSERT_GLYPH,
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
    private double driveRate = 0.5;
    private double armLiftHeight = 900;

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
            new MecanumNavigation.Navigation2D(24,0, degreesToRadians(0)),
            new MecanumNavigation.Navigation2D(24,24, degreesToRadians(90)),
            new MecanumNavigation.Navigation2D(0,24, degreesToRadians(180)),
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
                if (opMode.getEncoderValue(RobotHardware.MotorName.ARM_MOTOR) < initialArmEncoderTicks + armLiftHeight) {
                    opMode.setPower(RobotHardware.MotorName.ARM_MOTOR, 0.5);
                } else {
                    opMode.setPower(RobotHardware.MotorName.ARM_MOTOR, 0);
                }
            }
            // Give jewel arm 2.5 seconds to settle, then detect the color.
            if(stateTimer.seconds() > 2) {
                opMode.setPower(RobotHardware.MotorName.ARM_MOTOR, 0);
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

            waypointArray = generateWaypoints(teamColor,startPosition, opMode.glyphPositionVuMark, degreesToRadians(30));

            // Do the driving
            if(currentDriveWaypoint < waypointArray.size() ) {
                // Show Target Status and debug info
                opMode.telemetry.addData("Current Waypoint: ", currentDriveWaypoint);
                opMode.telemetry.addData("Target", waypointArray.get(currentDriveWaypoint).toString());
                arrivedAtWaypoint = opMode.autoDrive.rotateThenDriveToPosition(waypointArray.get(currentDriveWaypoint),driveRate);
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


    private ArrayList<MecanumNavigation.Navigation2D>
            generateWaypoints(Color.Ftc teamColor, RobotHardware.StartPosition startPosition,
                                RelicRecoveryVuMark positionVumark, double insertionSkewRadiansCCW) {
        ArrayList<MecanumNavigation.Navigation2D> waypointArray = new ArrayList<>(Arrays.asList(new MecanumNavigation.Navigation2D(0,0,0)));

        double dismountBlueDistance = 25;
        double dismountRedDistance = 25;
        double alignmentStrafeCorner = 12;
        double alignmentDriveCenter = 32;
        double approachCorner = 0;
        double approachCenter = 0;
        double insertCorner = 12;
        double insertCenter = 12;

        // Calculate trueSkew, skew with the appropriate sign.
        double trueSkewAngleRadiansCCW = 0;
        if (teamColor == Color.Ftc.BLUE) {
            if (startPosition == RobotHardware.StartPosition.FIELD_CENTER && opMode.glyphPositionVuMark == RelicRecoveryVuMark.RIGHT) {
                trueSkewAngleRadiansCCW = -1 * Math.abs(insertionSkewRadiansCCW);
            } else {
                trueSkewAngleRadiansCCW = Math.abs(insertionSkewRadiansCCW);
            }
        } else if(teamColor == Color.Ftc.RED) {
            if (startPosition == RobotHardware.StartPosition.FIELD_CENTER && opMode.glyphPositionVuMark == RelicRecoveryVuMark.LEFT) {
                trueSkewAngleRadiansCCW = Math.abs(insertionSkewRadiansCCW);
            } else {
                trueSkewAngleRadiansCCW = -1 * Math.abs(insertionSkewRadiansCCW);
            }
        }

        double alignmentOffsetRightTotal = getGlyphboxOffsetTowardRight(opMode.glyphPositionVuMark, trueSkewAngleRadiansCCW);
        double rotationInsertionCorrection = -getGlyphOffsetFromRotation(trueSkewAngleRadiansCCW).x;


        // Select Driving waypoints
        if (teamColor == Color.Ftc.BLUE) {
            if (startPosition == RobotHardware.StartPosition.FIELD_CORNER) {
                /** Blue Corner */
                waypointArray = new ArrayList<>(Arrays.asList(
                    // DISMOUNT
                    new MecanumNavigation.Navigation2D(dismountBlueDistance,
                            0, degreesToRadians(0)),
                    // ALIGN_W_OFFSETS
                    new MecanumNavigation.Navigation2D(dismountBlueDistance,
                            -alignmentStrafeCorner - alignmentOffsetRightTotal, degreesToRadians(0)),
                    // APPROACH (NULL)
                    new MecanumNavigation.Navigation2D(dismountBlueDistance + approachCorner,
                            -alignmentStrafeCorner - alignmentOffsetRightTotal, degreesToRadians(0)),
                    // ROTATE
                    new MecanumNavigation.Navigation2D(dismountBlueDistance,
                            -alignmentStrafeCorner - alignmentOffsetRightTotal, trueSkewAngleRadiansCCW),
                    // INSERT
                    new MecanumNavigation.Navigation2D(dismountBlueDistance + insertCorner + rotationInsertionCorrection,
                            -alignmentStrafeCorner - alignmentOffsetRightTotal, trueSkewAngleRadiansCCW)
                ));
            } else if (startPosition == RobotHardware.StartPosition.FIELD_CENTER) {
                /** Blue Center */
                waypointArray = new ArrayList<>(Arrays.asList(
                    // DISMOUNT
                    new MecanumNavigation.Navigation2D(dismountBlueDistance,
                            0, degreesToRadians(0)),
                    // ALIGN_W_OFFSETS
                    new MecanumNavigation.Navigation2D(dismountBlueDistance + alignmentDriveCenter,
                             0, degreesToRadians(0)),
                    // APPROACH (NULL)
                    new MecanumNavigation.Navigation2D(dismountBlueDistance + alignmentDriveCenter,
                            0, degreesToRadians(0)),
                    // ROTATE
                    new MecanumNavigation.Navigation2D(dismountBlueDistance + alignmentDriveCenter,
                            0 , trueSkewAngleRadiansCCW + degreesToRadians(90)),
                    // INSERT
                    new MecanumNavigation.Navigation2D(dismountBlueDistance + alignmentDriveCenter,
                            insertCenter + rotationInsertionCorrection, trueSkewAngleRadiansCCW + degreesToRadians(90))
                ));
            }
        } else if (teamColor == Color.Ftc.RED) {
            if (startPosition == RobotHardware.StartPosition.FIELD_CORNER) {
                /** Red Corner */
                waypointArray = new ArrayList<>(Arrays.asList(
                    // DISMOUNT
                    new MecanumNavigation.Navigation2D(-dismountRedDistance,
                            0, degreesToRadians(0)),
                    // ALIGN_W_OFFSETS
                    new MecanumNavigation.Navigation2D(-dismountRedDistance,
                            -alignmentStrafeCorner + alignmentOffsetRightTotal, degreesToRadians(0)),
                    // APPROACH (NULL)
                        new MecanumNavigation.Navigation2D(-dismountRedDistance,
                                -alignmentStrafeCorner + alignmentOffsetRightTotal, degreesToRadians(0)),
                    // ROTATE
                    new MecanumNavigation.Navigation2D(-dismountRedDistance,
                            -alignmentStrafeCorner + alignmentOffsetRightTotal, trueSkewAngleRadiansCCW + degreesToRadians(180)),
                    // INSERT
                    new MecanumNavigation.Navigation2D(-dismountRedDistance - insertCorner - rotationInsertionCorrection,
                            -alignmentStrafeCorner + alignmentOffsetRightTotal, trueSkewAngleRadiansCCW + degreesToRadians(180))
                ));
            } else if (startPosition == RobotHardware.StartPosition.FIELD_CENTER) {
                /** Red Center */
                waypointArray = new ArrayList<>(Arrays.asList(
                        // DISMOUNT
                        new MecanumNavigation.Navigation2D(-dismountRedDistance,
                                0, degreesToRadians(0)),
                        // ALIGN_W_OFFSETS
                        new MecanumNavigation.Navigation2D(-dismountRedDistance + -alignmentDriveCenter + alignmentOffsetRightTotal,
                                0, degreesToRadians(0)),
                        // APPROACH (NULL)
                        new MecanumNavigation.Navigation2D(-dismountRedDistance + -alignmentDriveCenter + alignmentOffsetRightTotal,
                                0, degreesToRadians(0)),
                        // ROTATE
                        new MecanumNavigation.Navigation2D(-dismountRedDistance + -alignmentDriveCenter + alignmentOffsetRightTotal,
                                0 , trueSkewAngleRadiansCCW + degreesToRadians(90)),
                        // INSERT
                        new MecanumNavigation.Navigation2D(-dismountRedDistance + -alignmentDriveCenter + alignmentOffsetRightTotal,
                                insertCenter + rotationInsertionCorrection, trueSkewAngleRadiansCCW + degreesToRadians(90))
                ));
            }
        } else {
            opMode.stopAllMotors();
            stateTimer.reset();
            state = AutoState.STATE_STOP;
        }

        return waypointArray;
    }




    private MecanumNavigation.Navigation2D getGlyphOffsetFromRotation(double rotationRadians) {
        double distanceToGlyphCenter = 12+6-7+1.5; // Added half of glyph width to lever arm.
        return new MecanumNavigation.Navigation2D( distanceToGlyphCenter * ( Math.cos(rotationRadians) - 1), distanceToGlyphCenter * Math.sin(rotationRadians), rotationRadians);
    }

    /**
     * Get the number of inches the robot needs to slide toward the right Cryptobox column
     * in order to line the glyph up with the desired column, at the desired skew angle.
     * @param vumarkPosition
     * @param skewAngleRadiansCCW
     * @return
     */
    private double getGlyphboxOffsetTowardRight( RelicRecoveryVuMark vumarkPosition, double skewAngleRadiansCCW) {
        MecanumNavigation.Navigation2D glyphOffsetFromRotation = getGlyphOffsetFromRotation(skewAngleRadiansCCW);
        double columnWidth = 6.5;
        double offsetRightTotal = 0;
        if (vumarkPosition == RelicRecoveryVuMark.LEFT) {
            offsetRightTotal -= columnWidth;
        } else if (vumarkPosition == RelicRecoveryVuMark.RIGHT) {
            offsetRightTotal += columnWidth;
        }
        offsetRightTotal += glyphOffsetFromRotation.y;

        return offsetRightTotal;
    }

}
