package org.firstinspires.ftc.teamcode.Utilities;

/**
 * Created by Ashley on 11/29/2017.
 */

public class Constants {

    public static final double MID_SERVO            =  0.5 ;

    // public doubles for the left and right CLAW
    public static final double LEFT_CLAW_OPEN = 0.25;
    public static final double RIGHT_CLAW_OPEN = 0.25;
    public static final double LEFT_CLAW_RELEASE = 0.08;
    public static final double RIGHT_CLAW_RELEASE = 0.08;

    public static final double LEFT_CLAW_CLOSED = 0.0;
    public static final double RIGHT_CLAW_CLOSED = 0.0;
    public static final double INITIAL_LEFT_CLAW_POS = 1.0;
    public static final double INITIAL_RIGHT_CLAW_POS = 1.0;

    public static final int RAISE_ARM_POSITION = 100;
    public static final int LOWER_ARM_POSITION = 50;

    public static final double RAISE_ARM_SPEED = 0.3;
    public static final double LOWER_ARM_SPEED = -0.3;

    public static final double DRIVE_WHEEL_DIAMETER_INCHES  =  4/2;

    public static final double MM_PER_IN = 25.4f;
    public static final double WHEELBASE_WIDTH_IN = 15.268;
    public static final double WHEELBASE_LENGTH_IN = 13.5;
    public static final double WHEELBASE_WIDTH_MM  = WHEELBASE_WIDTH_IN  * MM_PER_IN;
    public static final double WHEELBASE_LENGTH_MM  = WHEELBASE_LENGTH_IN  * MM_PER_IN;
    public static final double DRIVE_WHEEL_RADIUS_MM = DRIVE_WHEEL_DIAMETER_INCHES /2.0 * MM_PER_IN;
    public static final double DRIVE_WHEEL_MM_PER_ROT = DRIVE_WHEEL_RADIUS_MM * 2 *  Math.PI;

    public static final int DRIVE_WHEEL_STEPS_PER_ROT     = 28*40;

}
