package org.firstinspires.ftc.teamcode;

/**
 * Created by Ashley Trowell on 1/9/2018.
 */

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.teamcode.Utilities.AutoDeluxeStateMachine;
import org.firstinspires.ftc.teamcode.Utilities.AutoDrive;
import org.firstinspires.ftc.teamcode.Utilities.Color;
import org.firstinspires.ftc.teamcode.Utilities.Constants;
import org.firstinspires.ftc.teamcode.Utilities.Controller;
import org.firstinspires.ftc.teamcode.Utilities.MecanumNavigation;
import org.firstinspires.ftc.teamcode.Vision.SimpleVuforia;

/**
 * Autonomous mode for FTC Relic Recovery game.
 * Detects glyph target position and places it.
 */
public class AutoDeluxe extends RobotHardware {


    public MecanumNavigation mecanumNavigation;
    public AutoDrive autoDrive;
    protected Color.Ftc robotColor;
    protected StartPosition robotStartPos;
    protected AutoDeluxeStateMachine autoDeluxeStateMachine;
    protected SimpleVuforia vuforia;
    public RelicRecoveryVuMark glyphPositionVuMark = RelicRecoveryVuMark.UNKNOWN;
    private Thread thread;
    public Controller controller;
    public SkewMode skewMode = SkewMode.NORMAL;
    public enum SkewMode{
        NORMAL, NO_ARM_ENCODERS, T0, T90, TNEG90, T30, TNEG30,
    }


    @Autonomous(name="deluxe.Red.Center", group="deluxeAuto")
    public static class AutoRedCenter extends AutoDeluxe {
        @Override public void init() {
            robotColor = Color.Ftc.RED;
            robotStartPos = StartPosition.FIELD_CENTER;
            super.init();
        }
    }

    @Autonomous(name="deluxe.Red.Corner", group="deluxeAuto")
    public static class AutoRedCorner extends AutoDeluxe {
        @Override public void init() {
            robotColor = Color.Ftc.RED;
            robotStartPos = StartPosition.FIELD_CORNER;
            super.init();
        }
    }

    @Autonomous(name="deluxe.Blue.Center", group="deluxeAuto")
    public static class AutoBlueCenter extends AutoDeluxe {
        @Override public void init() {
            robotColor = Color.Ftc.BLUE;
            robotStartPos = StartPosition.FIELD_CENTER;
            super.init();
        }
    }

    @Autonomous(name="deluxe.Blue.Corner", group="deluxeAuto")
    public static class AutoBlueCorner extends AutoDeluxe {
        @Override public void init() {
            robotColor = Color.Ftc.BLUE;
            robotStartPos = StartPosition.FIELD_CORNER;
            super.init();
        }
    }

    @Override
    public void init() {
        super.init();
        controller = new Controller(gamepad1);
        thread = new Thread(new VuforiaLoader());
        thread.start();
        mecanumNavigation = new MecanumNavigation(this,
                new MecanumNavigation.DriveTrainMecanum(
                        Constants.WHEELBASE_LENGTH_IN, Constants.WHEELBASE_WIDTH_IN,
                        Constants.DRIVE_WHEEL_DIAMETER_INCHES, Constants.DRIVE_WHEEL_STEPS_PER_ROT,
                        Constants.DRIVE_WHEEL_LATERAL_RATIO));
        mecanumNavigation.initialize(new MecanumNavigation.Navigation2D(0, 0, 0),
                new MecanumNavigation.WheelTicks(getEncoderValue(MotorName.DRIVE_FRONT_LEFT),
                        getEncoderValue(MotorName.DRIVE_FRONT_RIGHT),
                        getEncoderValue(MotorName.DRIVE_BACK_LEFT),
                        getEncoderValue(MotorName.DRIVE_BACK_RIGHT)));
        autoDrive = new AutoDrive(this, mecanumNavigation);
        // Finally, construct the state machine.
        autoDeluxeStateMachine = new AutoDeluxeStateMachine(this, robotColor, robotStartPos);
        telemetry.addData("Initialization:", "Successful!");
    }

    @Override
    public void init_loop() {
        super.init_loop();
        controller.update();
        if (vuforia == null) {
            telemetry.addData("Vuforia:", "LOADING...");
        } else {
            telemetry.addData("Vuforia:", "INITIALIZED");
        }
        skewModeSelect();
        displayColorSensorTelemetry();
    }

    @Override
    public void start() {
        armServoTop();
        super.init();
        // Ensure starting position at origin, even if wheels turned since initialize.
        mecanumNavigation.update();
        mecanumNavigation.setCurrentPosition(new MecanumNavigation.Navigation2D(0,0,0));
        autoDeluxeStateMachine.init();
    }

    @Override
    public void loop() {
        super.loop();
        controller.update();
        mecanumNavigation.update();
        try {
            RelicRecoveryVuMark vuMark = vuforia.detectMark();
            setVumark(vuMark); // Store last non-UNKNOWN vumark detected.
            telemetry.addData("Vuforia Glyph Position", vuMark);
        } catch (Exception e) {
            telemetry.addData("Vuforia", "NOT INITIALIZED");
        }
        autoDeluxeStateMachine.update();
        mecanumNavigation.displayPosition();
        telemetry.addData("Current State", autoDeluxeStateMachine.state.toString());
        telemetry.addLine();
        displayColorSensorTelemetry();
    }

    private void setVumark(RelicRecoveryVuMark detectedVuMark) {
        if ( detectedVuMark != RelicRecoveryVuMark.UNKNOWN) {
            this.glyphPositionVuMark = detectedVuMark;
        }
    }

    // Initialize vuforia in a separate thread to avoid init() hangups.
    class VuforiaLoader implements Runnable {
        public void run() {
            vuforia = new SimpleVuforia(getVuforiaLicenseKey(), AutoDeluxe.this, false, false);
        }
    }

    private void skewModeSelect(){
        // Select control mode with dpad left and right.
        telemetry.addData("Select Control Mode:","");
        telemetry.addData("Control Mode:", skewMode.toString());
        if(controller.dpadRightOnce()) {
            if( skewMode.ordinal() >=  SkewMode.values().length -1) {
                // If at max value, loop to first.
                skewMode = SkewMode.values()[0];
            } else {
                // Go to next control mode
                skewMode = SkewMode.values()[skewMode.ordinal() + 1];
            }
        } else if ( controller.dpadLeftOnce()) {
            if (skewMode.ordinal() <= 0) {
                // If at first value, loop to last value
                skewMode = SkewMode.values()[SkewMode.values().length-1];
            } else {
                // Go to previous control mode
                skewMode = SkewMode.values()[skewMode.ordinal() - 1];
            }
        }
    }
}

