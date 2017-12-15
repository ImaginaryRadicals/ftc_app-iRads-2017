package org.firstinspires.ftc.teamcode;

/**
 * Created by ryderswan on 12/14/17.
 */

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Utilities.Color;



import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.teamcode.Utilities.Color;

/**
 * Autonomous demo for FTC Relic Recovery game.
 */
public class Auto extends RobotHardware {



    protected Color.Ftc robotColor;
    protected StartPosition robotStartPos;


    @Autonomous(name="pmt.Red.Center", group="pmtischler")
    public static class AutoRedCenter extends Auto {
        @Override public void init() {
            robotColor = Color.Ftc.RED;
            robotStartPos = StartPosition.FIELD_CENTER;
            super.init();
        }
    }

    @Autonomous(name="pmt.Red.Corner", group="pmtischler")
    public static class AutoRedCorner extends Auto {
        @Override public void init() {
            robotColor = Color.Ftc.RED;
            robotStartPos = StartPosition.FIELD_CORNER;
            super.init();
        }
    }

    @Autonomous(name="pmt.Blue.Center", group="pmtischler")
    public static class AutoBlueCenter extends Auto {
        @Override public void init() {
            robotColor = Color.Ftc.BLUE;
            robotStartPos = StartPosition.FIELD_CENTER;
            super.init();
        }
    }

    @Autonomous(name="pmt.Blue.Corner", group="pmtischler")
    public static class AutoBlueCorner extends Auto {
        @Override public void init() {
            robotColor = Color.Ftc.BLUE;
            robotStartPos = StartPosition.FIELD_CORNER;
            super.init();
        }
    }

    @Override
    public void init() {
        super.init();

    }

    @Override
    public void loop() {

    }
}
