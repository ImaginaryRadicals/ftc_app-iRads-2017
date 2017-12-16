package org.firstinspires.ftc.teamcode.Utilities;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotHardware;

/**
 * Created by Ashley on 12/16/2017.
 */

public class AutoSimpleJewelStateMachine {

    public enum JewelState {
        STATE_LOWER_ARM,
        STATE_DETECT_COLOR,
        STATE_HIT_DETECTED_JEWEL_CW,
        STATE_HIT_OTHER_JEWEL_CCW,
        STATE_ABORT_JEWEL,
        STATE_DRIVE_TO_GLYPH_BOX,
        STATE_STOP,
    }

    public JewelState state = JewelState.STATE_LOWER_ARM;
    private RobotHardware opMode;
    private ElapsedTime stateLoopTimer = new ElapsedTime();
    private double lastStateLoopPeriod = 0;
    private ElapsedTime stateTimer = new ElapsedTime();

    // Colors
    public Color.Ftc teamColor = Color.Ftc.UNKNOWN;
    public Color.Ftc jewelColor = Color.Ftc.UNKNOWN;

    // Kinematics
    private double timeHitJewel = 1;
    private double powerHitJewel = 0.2;
    private double timeParkDrive = 1.5;
    private double powerParkDrive = 1;
    private double steeringRatioParkDrive = 0.5;
    public RobotHardware.StartPosition startPosition;


    public AutoSimpleJewelStateMachine(RobotHardware opMode, Color.Ftc teamColor, RobotHardware.StartPosition startPosition) {
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


        if (state == JewelState.STATE_LOWER_ARM) {
            opMode.armServoBottom();
            opMode.closeClaw();

            if (opMode.getEncoderValue(RobotHardware.MotorName.ARM_MOTOR) < 500) {
                opMode.setPower(RobotHardware.MotorName.ARM_MOTOR, 0.5);
            } else {
                opMode.setPower(RobotHardware.MotorName.ARM_MOTOR, 0);
            }
            
            if(stateTimer.seconds() > 4) {
                state = JewelState.STATE_DETECT_COLOR;
                stateTimer.reset();
            }

        } else if (state == JewelState.STATE_DETECT_COLOR) {

            setJewelColor( opMode.getJewelColor() );

            if(stateTimer.seconds() > 2) {

                stateTimer.reset();
                if (jewelColor == teamColor) {
                    state = JewelState.STATE_HIT_OTHER_JEWEL_CCW;
                } else if (jewelColor == Color.Ftc.UNKNOWN) {
                    state = JewelState.STATE_ABORT_JEWEL;
                } else {
                    state = JewelState.STATE_HIT_DETECTED_JEWEL_CW;
                }
            }
        } else if (state == JewelState.STATE_HIT_DETECTED_JEWEL_CW) {
            if(stateTimer.seconds() < timeHitJewel) {
                opMode.setDriveForTank(powerHitJewel,-powerHitJewel);
            } else if (stateTimer.seconds() < 2*timeHitJewel ) {
                opMode.armServoTop();
                opMode.setDriveForTank(-powerHitJewel,powerHitJewel);
            } else {
                opMode.stopAllMotors();
                stateTimer.reset();
                state = JewelState.STATE_DRIVE_TO_GLYPH_BOX;
            }
        } else if (state == JewelState.STATE_HIT_OTHER_JEWEL_CCW) {
            if(stateTimer.seconds() < timeHitJewel) {
                opMode.setDriveForTank(-powerHitJewel,powerHitJewel);
            } else if (stateTimer.seconds() < 2*timeHitJewel ) {
                opMode.armServoTop();
                opMode.setDriveForTank(powerHitJewel,-powerHitJewel);
            } else {
                opMode.stopAllMotors();
                stateTimer.reset();
                state = JewelState.STATE_DRIVE_TO_GLYPH_BOX;
            }

        } else if (state == JewelState.STATE_ABORT_JEWEL) {
            opMode.armServoTop();
            if (stateTimer.seconds() > 1) {
                state = JewelState.STATE_DRIVE_TO_GLYPH_BOX;
                stateTimer.reset();
            }
        } else if (state == JewelState.STATE_DRIVE_TO_GLYPH_BOX) {
            double direction, powerLeft, powerRight;

            // Calculate driving motion
            if (teamColor == Color.Ftc.BLUE) {
                direction = 1;
            } else if (teamColor == Color.Ftc.RED) {
                direction = -1;
            } else {
                direction = 0;
                opMode.stopAllMotors();
                stateTimer.reset();
                state = JewelState.STATE_STOP;
            }

            powerLeft = direction * powerParkDrive * 1;
            powerRight = direction * powerParkDrive * 1;

            // Add steering
            if (startPosition == RobotHardware.StartPosition.FIELD_CORNER) {
                // Left Wheel fast, Right wheel slow
                powerRight *= steeringRatioParkDrive;
            } else if (startPosition == RobotHardware.StartPosition.FIELD_CENTER) {
                // Right Wheel fast, Left wheel slow
                powerLeft *= steeringRatioParkDrive;
            }

            // Do the driving!
            if(stateTimer.seconds() < timeParkDrive) {
                opMode.setDriveForTank(powerLeft, powerRight);
            } else { // Next State Logic
                stateTimer.reset();
                opMode.stop();
                state = JewelState.STATE_STOP;
            }
        } else if (state == JewelState.STATE_STOP) {
            opMode.stopAllMotors();
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






}
