package org.firstinspires.ftc.teamcode.Utilities;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.teamcode.AutoDeluxe;
import org.firstinspires.ftc.teamcode.RobotHardware;

import java.util.ArrayList;
import java.util.Arrays;

/**
 * Created by Ashley on 12/16/2017.
 */

public class AutoDeluxeStateMachine {

    public enum AutoState {
        STATE_START,
        STATE_LOWER_JEWEL_ARM_AND_LIFT_CLAW,
        STATE_DETECT_COLOR,
        STATE_HIT_DETECTED_JEWEL_CW,
        STATE_HIT_OTHER_JEWEL_CCW,
        STATE_ABORT_JEWEL,
        STATE_FIND_VUMARK_AND_GENERATE_WAYPOINTS,
        STATE_DISMOUNT,
        STATE_ALIGN_W_OFFSETS,
        STATE_APPROACH_GLYPH_BOX,
        STATE_ROTATE_BEFORE_INSERT_THEN_OPEN_CLAW, // lower arm and open claw
        STATE_INSERT_GLYPH,
        STATE_BACKUP,
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
    private ArrayList<MecanumNavigation.Navigation2D> waypointArrayGlobal;
    private double timeHitJewel = 0.5;
    private double powerHitJewel = 0.2;
    public RobotHardware.StartPosition startPosition;
    private int initialArmEncoderTicks = 0;
    private int currentDriveWaypoint = 0;
    private double driveRate = 0.5;
    private double armLiftHeight = 700;
    private double signedSkewAngleRadiansCCW;


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
            state = AutoState.STATE_LOWER_JEWEL_ARM_AND_LIFT_CLAW;
        } else if (state == AutoState.STATE_LOWER_JEWEL_ARM_AND_LIFT_CLAW) {
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
                state = AutoState.STATE_FIND_VUMARK_AND_GENERATE_WAYPOINTS;
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
                state = AutoState.STATE_FIND_VUMARK_AND_GENERATE_WAYPOINTS;
            }

        } else if (state == AutoState.STATE_ABORT_JEWEL) {
            opMode.armServoTop();
            if (stateTimer.seconds() > 1) {
                state = AutoState.STATE_FIND_VUMARK_AND_GENERATE_WAYPOINTS;
                stateTimer.reset();
            }
        } else if (state == AutoState.STATE_FIND_VUMARK_AND_GENERATE_WAYPOINTS) {
            if (opMode.glyphPositionVuMark == RelicRecoveryVuMark.UNKNOWN && stateTimer.seconds() < 2) {
                opMode.autoDrive.rotateThenDriveToPosition(new MecanumNavigation.Navigation2D(0,0,degreesToRadians(20)),driveRate);
            } else {
                this.waypointArrayGlobal = generateWaypoints(teamColor,startPosition, opMode.glyphPositionVuMark, degreesToRadians(30));
                state = AutoState.STATE_DISMOUNT;
                stateTimer.reset();
            }
        } else if (state == AutoState.STATE_DISMOUNT) {
            boolean arrivedAtWaypoint = false;
            arrivedAtWaypoint = driveToWaypointAtRate(0,driveRate);
            if (arrivedAtWaypoint) {
                state = AutoState.STATE_ALIGN_W_OFFSETS;
                stateTimer.reset();
            }
        } else if (state == AutoState.STATE_ALIGN_W_OFFSETS) {
            boolean arrivedAtWaypoint = false;
            arrivedAtWaypoint = driveToWaypointAtRate(1,driveRate);
            if (arrivedAtWaypoint) {
                state = AutoState.STATE_ROTATE_BEFORE_INSERT_THEN_OPEN_CLAW;
                stateTimer.reset();
            }
        } else if (state == AutoState.STATE_APPROACH_GLYPH_BOX) {
            boolean arrivedAtWaypoint = false;
            arrivedAtWaypoint = driveToWaypointAtRate(2,driveRate);
            if (arrivedAtWaypoint) {
                state = AutoState.STATE_ROTATE_BEFORE_INSERT_THEN_OPEN_CLAW;
                stateTimer.reset();
            }
        } else if (state == AutoState.STATE_ROTATE_BEFORE_INSERT_THEN_OPEN_CLAW) {
            boolean arrivedAtWaypoint = false;
            boolean armLowered = false;
            arrivedAtWaypoint = driveToWaypointAtRate(3,driveRate);

            if (arrivedAtWaypoint) {
                // Lower arm
                if ( opMode.getEncoderValue(RobotHardware.MotorName.ARM_MOTOR) > initialArmEncoderTicks) {
                    opMode.setPower(RobotHardware.MotorName.ARM_MOTOR,-0.2);
                    stateTimer.reset();
                } else {
                    opMode.setPower(RobotHardware.MotorName.ARM_MOTOR, 0);
                    opMode.stopAllMotors();
                    armLowered = true;
                }


                if (armLowered) {
                    // Claw logic
                    if(signedSkewAngleRadiansCCW > 0) {
                        // Open right servo claw
                        opMode.setAngle(RobotHardware.ServoName.CLAW_RIGHT,Constants.INITIAL_RIGHT_CLAW_POS);
                    } else if(signedSkewAngleRadiansCCW < 0) {
                        // Open left servo claw
                        opMode.setAngle(RobotHardware.ServoName.CLAW_LEFT,Constants.INITIAL_LEFT_CLAW_POS);
                    } else {
                        // Open both claws
                        opMode.setAngle(RobotHardware.ServoName.CLAW_LEFT,Constants.INITIAL_LEFT_CLAW_POS);
                        opMode.setAngle(RobotHardware.ServoName.CLAW_RIGHT,Constants.INITIAL_RIGHT_CLAW_POS);
                    }

                    if (stateTimer.seconds() > 0.5) {
                        // Next state logic
                        opMode.stopAllMotors();
                        state = AutoState.STATE_INSERT_GLYPH;
                        stateTimer.reset();
                    }
                }
            }
        } else if (state == AutoState.STATE_INSERT_GLYPH) {
            boolean arrivedAtWaypoint = false;
            arrivedAtWaypoint = driveToWaypointAtRate(4,driveRate);
            if (arrivedAtWaypoint || stateTimer.seconds() > 4) {
                state = AutoState.STATE_BACKUP;
                stateTimer.reset();
            }
        } else if (state == AutoState.STATE_BACKUP) {
            boolean arrivedAtWaypoint = false;
            arrivedAtWaypoint = driveToWaypointAtRate(5,driveRate);

            // Open both claws
            opMode.setAngle(RobotHardware.ServoName.CLAW_LEFT,Constants.INITIAL_LEFT_CLAW_POS);
            opMode.setAngle(RobotHardware.ServoName.CLAW_RIGHT,Constants.INITIAL_RIGHT_CLAW_POS);

            if (arrivedAtWaypoint) {
                state = AutoState.STATE_STOP;
                stateTimer.reset();
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

        double dismountBlueDistance = 24;
        double dismountRedDistance = 25;
        double alignmentStrafeCorner = 12;
        double alignmentDriveCenter = 22 - 6 + 2 - 6.25;
        double approachCorner = 0;
        double approachCenter = 0;
        double insertCorner = 12;
        double insertCenter = 12;
        double backupDistance = 4;

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

        if (opMode.skewMode == AutoDeluxe.SkewMode.NORMAL) {

        } else if(opMode.skewMode == AutoDeluxe.SkewMode.T0) {
            trueSkewAngleRadiansCCW = 0;
        } else if(opMode.skewMode == AutoDeluxe.SkewMode.T90) {
            trueSkewAngleRadiansCCW = 90;
        } else if(opMode.skewMode == AutoDeluxe.SkewMode.TNEG90) {
            trueSkewAngleRadiansCCW = -90;
        } else if(opMode.skewMode == AutoDeluxe.SkewMode.T30) {
            trueSkewAngleRadiansCCW = 30;
        } else if (opMode.skewMode == AutoDeluxe.SkewMode.TNEG30) {
            trueSkewAngleRadiansCCW = -30;
        }

        this.signedSkewAngleRadiansCCW = trueSkewAngleRadiansCCW;

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
                            -alignmentStrafeCorner - alignmentOffsetRightTotal, trueSkewAngleRadiansCCW),
                    // BACKUP
                    new MecanumNavigation.Navigation2D(dismountBlueDistance + insertCorner + rotationInsertionCorrection - backupDistance,
                            -alignmentStrafeCorner - alignmentOffsetRightTotal, trueSkewAngleRadiansCCW)
                ));
            } else if (startPosition == RobotHardware.StartPosition.FIELD_CENTER) {
                /** Blue Center */
                waypointArray = new ArrayList<>(Arrays.asList(
                    // DISMOUNT
                    new MecanumNavigation.Navigation2D(dismountBlueDistance,
                            0, degreesToRadians(0)),
                    // ALIGN_W_OFFSETS
                    new MecanumNavigation.Navigation2D(dismountBlueDistance + alignmentDriveCenter + alignmentOffsetRightTotal,
                             0, degreesToRadians(0)),
                    // APPROACH (NULL)
                    new MecanumNavigation.Navigation2D(dismountBlueDistance + alignmentDriveCenter + alignmentOffsetRightTotal,
                            0, degreesToRadians(0)),
                    // ROTATE
                    new MecanumNavigation.Navigation2D(dismountBlueDistance + alignmentDriveCenter + alignmentOffsetRightTotal,
                            0 , trueSkewAngleRadiansCCW + degreesToRadians(90)),
                    // INSERT
                    new MecanumNavigation.Navigation2D(dismountBlueDistance + alignmentDriveCenter + alignmentOffsetRightTotal,
                            insertCenter + rotationInsertionCorrection, trueSkewAngleRadiansCCW + degreesToRadians(90)),
                    // BACKUP
                    new MecanumNavigation.Navigation2D(dismountBlueDistance + alignmentDriveCenter + alignmentOffsetRightTotal,
                            insertCenter + rotationInsertionCorrection - backupDistance, trueSkewAngleRadiansCCW + degreesToRadians(90))
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
                            -alignmentStrafeCorner + alignmentOffsetRightTotal, trueSkewAngleRadiansCCW + degreesToRadians(180)),
                    // BACKUP
                    new MecanumNavigation.Navigation2D(-dismountRedDistance - insertCorner - rotationInsertionCorrection + backupDistance,
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
                                insertCenter + rotationInsertionCorrection, trueSkewAngleRadiansCCW + degreesToRadians(90)),
                        // BACKUP
                        new MecanumNavigation.Navigation2D(-dismountRedDistance + -alignmentDriveCenter + alignmentOffsetRightTotal,
                                insertCenter + rotationInsertionCorrection - backupDistance, trueSkewAngleRadiansCCW + degreesToRadians(90))
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

    private boolean driveToWaypointAtRate(int waypointNumber, double driveRate) {
        // Show Target Status and debug info
        opMode.telemetry.addData("Current Waypoint: ", currentDriveWaypoint);
        opMode.telemetry.addData("Target", waypointArrayGlobal.get(waypointNumber).toString());
        return opMode.autoDrive.rotateThenDriveToPosition(waypointArrayGlobal.get(waypointNumber),driveRate);
    }

}
