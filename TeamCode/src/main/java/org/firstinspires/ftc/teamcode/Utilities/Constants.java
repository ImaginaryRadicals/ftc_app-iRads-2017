package org.firstinspires.ftc.teamcode.Utilities;

/**
 * Created by Ashley on 11/29/2017.
 */

public class Constants {

    public static final double MID_SERVO            =  0.5 ;

    // public doubles for the left and right GRIPPER
    public static final double LEFT_GRIPPER_OPEN = 0.0;
    public static final double LEFT_GRIPPER_CLOSED = 0.66;
    public static final double RIGHT_GRIPPER_OPEN = 0.0;
    public static final double RIGHT_GRIPPER_CLOSED = 0.66;
    public static final double INITIAL_LEFT_GRIPPER_POS = LEFT_GRIPPER_OPEN;
    public static final double INITIAL_RIGHT_GRIPPER_POS = RIGHT_GRIPPER_OPEN;

    public static final double DRIVE_WHEEL_DIAMETER_INCHES  =  4;

    public static final double MM_PER_IN = 25.4f;
    public static final double WHEELBASE_WIDTH_IN = 18;
    public static final double WHEELBASE_LENGTH_IN = 13;
    public static final double WHEELBASE_WIDTH_MM  = WHEELBASE_WIDTH_IN  * MM_PER_IN;
    public static final double WHEELBASE_LENGTH_MM  = WHEELBASE_LENGTH_IN  * MM_PER_IN;
    public static final double DRIVE_WHEEL_RADIUS_MM = DRIVE_WHEEL_DIAMETER_INCHES /2.0 * MM_PER_IN;
    public static final double DRIVE_WHEEL_MM_PER_ROT = DRIVE_WHEEL_RADIUS_MM * 2 *  Math.PI;

    public static final int DRIVE_WHEEL_STEPS_PER_ROT     = 28*40;

}
