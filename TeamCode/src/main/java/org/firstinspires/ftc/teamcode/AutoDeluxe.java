package org.firstinspires.ftc.teamcode;

/**
 * Created by Ashley Trowell on 1/9/2018.
 */

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.teamcode.Utilities.AutoDeluxeStateMachine;
import org.firstinspires.ftc.teamcode.Utilities.AutoSimpleJewelStateMachine;
import org.firstinspires.ftc.teamcode.Utilities.Color;
import org.firstinspires.ftc.teamcode.Vision.SimpleVuforia;

/**
 * Autonomous mode for FTC Relic Recovery game.
 * Detects glyph target position and places it.
 */
public class AutoDeluxe extends RobotHardware {



    protected Color.Ftc robotColor;
    protected StartPosition robotStartPos;
    protected AutoDeluxeStateMachine autoDeluxeStateMachine;
    private SimpleVuforia vuforia;


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
        autoDeluxeStateMachine = new AutoDeluxeStateMachine(this, vuforia, robotColor, robotStartPos);
    }

    @Override
    public void start() {
        armServoTop();
        super.init();
        vuforia = new SimpleVuforia(getVuforiaLicenseKey(), this, true, false);
        autoDeluxeStateMachine.init();
    }

    @Override
    public void loop() {
        super.loop();
        RelicRecoveryVuMark vuMark = vuforia.detectMark();
        telemetry.addData("Vuforia Glyph Position", vuMark);
        autoDeluxeStateMachine.update();

    }
}
