package org.firstinspires.ftc.teamcode.Utilities;

/**
 * Created by Ashley on 11/29/2017.
 */

public class Constants {

    public static final double MID_SERVO            =  0.5 ;

    // public doubles for the left and right CLAW
    public static final double LEFT_CLAW_OPEN = 0.31;
    public static final double RIGHT_CLAW_OPEN = 0.31;
    public static final double LEFT_CLAW_RELEASE = 0.07;
    public static final double RIGHT_CLAW_RELEASE = 0.07;

    public static final double LEFT_CLAW_CLOSED = 0.00;
    public static final double RIGHT_CLAW_CLOSED = 0.00;
    public static final double INITIAL_LEFT_CLAW_POS = 0.65;
    public static final double INITIAL_RIGHT_CLAW_POS = 0.65;

    public static final double JEWEL_ARM_INITIAL = 0.97;
    public static final double JEWEL_ARM_BOTTOM = 0.05;
    public static final double JEWEL_ARM_TOP = 0.81;

    public static final double ARM_TICKS_PER_DEGREE = 5;
    public static final int RAISE_ARM_POSITION = 3000;
    public static final int LOWER_ARM_POSITION = 0;


    public static final double DRIVE_WHEEL_DIAMETER_INCHES  =  4;
    public static final double DRIVE_WHEEL_LATERAL_RATIO = 0.89;

    public static final double MM_PER_IN = 25.4f;
    private static double rotationScaleIncrease = 1.0975;
    public static final double WHEELBASE_WIDTH_IN = 15.268 / rotationScaleIncrease;
    public static final double WHEELBASE_LENGTH_IN = 13.5 / rotationScaleIncrease;
    public static final double WHEELBASE_WIDTH_MM  = WHEELBASE_WIDTH_IN  * MM_PER_IN;
    public static final double WHEELBASE_LENGTH_MM  = WHEELBASE_LENGTH_IN  * MM_PER_IN;
    public static final double DRIVE_WHEEL_RADIUS_MM = DRIVE_WHEEL_DIAMETER_INCHES /2.0 * MM_PER_IN;
    public static final double DRIVE_WHEEL_MM_PER_ROT = DRIVE_WHEEL_RADIUS_MM * 2 *  Math.PI;

    public static final int DRIVE_WHEEL_STEPS_PER_ROT     = 28*40;

}
