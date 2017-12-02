package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Utilities.VectorMath;
import org.firstinspires.ftc.teamcode.Utilities.Constants;

import java.util.Vector;

/**
 *
 * This class can be used to define all the specific hardware for a single robot.
 */
public class RobotHardware
{
    /* Public members. */
    public DcMotor  leftDriveMotor      = null;
    public DcMotor  rightDriveMotor     = null;
    public DcMotor  leftLaunchMotor     = null;
    public DcMotor  rightLaunchMotor    = null;
    public DcMotor  liftMotor           = null;

    public Servo    leftGripper         = null;
    public Servo    rightGripper        = null;

    public static boolean hardwareEnabled = true; // set false if hardware errors are encountered

    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    Telemetry   telemetry = null;
    private ElapsedTime period  = new ElapsedTime();
    private Vector pastPeriods  = new Vector();


    /* Constructor */
    public RobotHardware(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap, Telemetry telemetry) {
        // Save reference to Hardware map
        hwMap = ahwMap;
        this.telemetry = telemetry;

        // Define and Initialize Motors
        leftDriveMotor    = hwMap.dcMotor.get("leftDriveMotor");
        rightDriveMotor   = hwMap.dcMotor.get("rightDriveMotor");
        leftLaunchMotor   = hwMap.dcMotor.get("leftLaunchMotor");
        rightLaunchMotor  = hwMap.dcMotor.get("rightLaunchMotor");
        liftMotor         = hwMap.dcMotor.get("liftMotor");
        // Set Motor Direction
        leftDriveMotor.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        rightDriveMotor.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        leftLaunchMotor.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        rightLaunchMotor.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors

        // Set all motors to zero power
        leftDriveMotor.setPower(0);
        rightDriveMotor.setPower(0);
        leftLaunchMotor.setPower(0);
        rightLaunchMotor.setPower(0);
        liftMotor.setPower(0);


        // Set all motors to run with encoders.
        // Use RUN_WITHOUT_ENCODERS if encoders are NOT installed.
        // Use RUN_USING_ENCODERS if encoders ARE installed.
        leftDriveMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDriveMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftLaunchMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightLaunchMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        // Allow launch wheels to coast.
        leftLaunchMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightLaunchMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);



        // Define and initialize ALL installed servos.
        try {

            leftGripper = hwMap.servo.get("leftGripper");
            leftGripper.setPosition(INITIAL_LEFT_GRIPPER_POS);
        } catch (Exception e) {
            e.getMessage()
        }

        try {
            rightGripper = hwMap.servo.get("rightGripper");
            rightGripper.setPosition(INITIAL_RIGHT_GRIPPER_POS);
            rightGripper.setDirection(Servo.Direction.REVERSE);
        } catch (Exception e ) {

        }


    }

    /***
     *
     * waitForTick implements a periodic delay. However, this acts like a metronome with a regular
     * periodic tick.  This is used to compensate for varying processing times for each cycle.
     * The function looks at the elapsed cycle time, and sleeps for the remaining time interval.
     *
     * @param periodMs  Length of wait cycle in mSec.
     */
    public void waitForTick(long periodMs) {

        long  remaining = periodMs - (long)period.milliseconds();

        // sleep for the remaining portion of the regular cycle period.
        if (remaining > 0) {
            try {
                Thread.sleep(remaining);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }

        // Reset the cycle clock for the next pass.
        period.reset();
    }

    public double periodSec(){

        pastPeriods.add(period.time());
        period.reset();
        if (pastPeriods.size()>= 30) {
            pastPeriods.remove(0);
        }
        return VectorMath.average(pastPeriods);
    }
}

