package org.firstinspires.ftc.teamcode.Utilities;

/**
 * Created by Ashley on 11/29/2017.
 */

public class Constants {

    // public doubles for the left and right CLAW
    public static final double LEFT_CLAW_OPEN = 0.39;
    public static final double RIGHT_CLAW_OPEN = 0.49;
    public static final double LEFT_CLAW_RELEASE = 0.22;
    public static final double RIGHT_CLAW_RELEASE = 0.32;

    public static final double LEFT_CLAW_CLOSED = 0.05;
    public static final double RIGHT_CLAW_CLOSED = 0.15;
    public static final double INITIAL_LEFT_CLAW_POS = 0.73;
    public static final double INITIAL_RIGHT_CLAW_POS = 0.83;

    public static final double JEWEL_ARM_INITIAL = 0.97;
    public static final double JEWEL_ARM_BOTTOM = 0.05;
    public static final double JEWEL_ARM_TOP = 0.81;

    // Arm geometry
    public static final double ARM_LENGTH_INCHES = 18; // ?
    public static final double ARM_BOTTOM_ANGLE_DEGREES = -40.5;
    public static final double ARM_TOP_ANGLE_DEGREES = 38;
    public static final int ARM_BOTTOM_TICKS = 0;
    public static final int ARM_LEVEL_TICKS = 500; // ?
    public static final int ARM_TOP_TICKS = 1000; // ?
    public static final double ARM_SLOP_DEGREES = 15; // ?
    public static final double ARM_SLOP_TICKS = 100; // ?

    // Block stacking geometry
    public static final double BLOCK_HEIGHT_INCHES = 6;
    public static final double BLOCK_OFFSET_INCHES = 1;

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
