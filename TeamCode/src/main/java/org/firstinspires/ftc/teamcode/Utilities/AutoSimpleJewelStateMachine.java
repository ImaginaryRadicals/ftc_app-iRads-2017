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
    private double moveTime = 1;
    private double movePower = 0.2;


    public AutoSimpleJewelStateMachine(RobotHardware opMode, Color.Ftc teamColor) {
        this.opMode = opMode;
        this.teamColor = teamColor;
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
            
            if(stateTimer.seconds() > 2) {
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
            if(stateTimer.seconds() < moveTime) {
                opMode.setDriveForTank(movePower,-movePower);
            } else if (stateTimer.seconds() < 2*moveTime ) {
                opMode.setDriveForTank(-movePower,movePower);
            } else {
                opMode.stopAllMotors();
                stateTimer.reset();
                state = JewelState.STATE_STOP;
            }
        } else if (state == JewelState.STATE_HIT_OTHER_JEWEL_CCW) {
            if(stateTimer.seconds() < moveTime) {
                opMode.setDriveForTank(-movePower,movePower);
            } else if (stateTimer.seconds() < 2*moveTime ) {
                opMode.setDriveForTank(movePower,-movePower);
            } else {
                opMode.stopAllMotors();
                stateTimer.reset();
                state = JewelState.STATE_STOP;
            }
            
        } else if (state == JewelState.STATE_ABORT_JEWEL) {
            opMode.armServoTop();
            if(stateTimer.seconds() > 1) {
                state = JewelState.STATE_STOP;
                stateTimer.reset();
            }

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
