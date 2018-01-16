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
    private SimpleVuforia vuforia;
    public RelicRecoveryVuMark glyphPositionVuMark;


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
        vuforia = new SimpleVuforia(getVuforiaLicenseKey(), this, false, false);
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
    public void start() {
        armServoTop();
        super.init();
        autoDeluxeStateMachine.init();
    }

    @Override
    public void loop() {
        super.loop();
        mecanumNavigation.update();
        RelicRecoveryVuMark vuMark = vuforia.detectMark();
        setVumark(vuMark); // Store last non-UNKNOWN vumark detected.
        telemetry.addData("Vuforia Glyph Position", vuMark);
        autoDeluxeStateMachine.update();
        mecanumNavigation.displayPosition();
        telemetry.addData("Current State", autoDeluxeStateMachine.state.toString());
    }

    private void setVumark(RelicRecoveryVuMark detectedVuMark) {
        if ( detectedVuMark != RelicRecoveryVuMark.UNKNOWN) {
            this.glyphPositionVuMark = detectedVuMark;
        }
    }
}

