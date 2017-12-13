package org.firstinspires.ftc.teamcode.Utilities;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

/**
 * Created by Ashley on 12/8/2017.
 */

public class MecanumNavigation {

    private OpMode opMode;
    public DriveTrainMecanum driveTrainMecanum;
    public Navigation2D currentPosition;
    public Navigation2D previousPosition;
    public WheelTicks wheelTicks;


    public MecanumNavigation(OpMode opMode, DriveTrainMecanum driveTrainMecanum) {
        this.opMode = opMode;
        this.driveTrainMecanum = driveTrainMecanum;
    }

    public void initialize(Navigation2D initialNavPosition, WheelTicks initialWheelTicks) {
        currentPosition = initialNavPosition;
        wheelTicks = initialWheelTicks;
    }

    public void setCurrentPosition(Navigation2D currentPosition) {
        this.currentPosition = currentPosition;
    }

    public void update(WheelTicks newWheelTicks) {
        WheelTicks deltaWheelTicks = newWheelTicks.getDeltaFromPrevious(wheelTicks);
        this.wheelTicks = (WheelTicks)newWheelTicks.clone();

        Navigation2D deltaPosition = deltaPositionFromDeltaWheelTicks(deltaWheelTicks);
        this.previousPosition = this.currentPosition;
        this.currentPosition.addRelativeDeltaToAbsolute(deltaPosition);
    }

    public void displayPosition() {
        opMode.telemetry.addData("X: ", currentPosition.x);
        opMode.telemetry.addData("Y: ", currentPosition.y);
        opMode.telemetry.addData("Theta: ", currentPosition.theta * 180 / Math.PI);
    }

    public Navigation2D deltaPositionFromDeltaWheelTicks(WheelTicks deltaWheelTicks) {
        double wheelRadius = driveTrainMecanum.wheelDiameter/2;
        double wheelbaseK = driveTrainMecanum.getK();

        double frontLeftRadians = driveTrainMecanum.ticksToRadians(deltaWheelTicks.frontLeft);
        double frontRightRadians = driveTrainMecanum.ticksToRadians(deltaWheelTicks.frontRight);
        double backLeftRadians = driveTrainMecanum.ticksToRadians(deltaWheelTicks.backLeft);
        double backRightRadians = driveTrainMecanum.ticksToRadians(deltaWheelTicks.backRight);

        double R_4 = wheelRadius / 4;

        double deltaX = (frontLeftRadians + frontRightRadians + backLeftRadians + backRightRadians)
                * R_4;
        double deltaY = (-frontLeftRadians + frontRightRadians + backLeftRadians - backRightRadians)
                * R_4;
        double deltaTheta = (-frontLeftRadians + frontRightRadians - backLeftRadians + backRightRadians)
                * R_4 / wheelbaseK;

        return new Navigation2D(deltaX,deltaY,deltaTheta);
    }



    /**
     * 2d position plus angular orientation.
     */
    public static class Navigation2D{
        public double x = 0;
        public double y = 0;
        // Rotation degrees CCW
        public double theta = 0;

        public Navigation2D(double x, double y, double theta) {
            this.x = x;
            this.y = y;
            this.theta = theta;
        }

        public void add (Navigation2D other) {
            this.x += other.x;
            this.y += other.y;
            this.theta += other.theta;
        }

        public void subtract (Navigation2D other) {
            this.x -= other.x;
            this.y -= other.y;
            this.theta -= other.theta;
        }

        /**
         * Returns this object minus argument.
         * @param other Navigation2D object
         * @return this minus argument
         */
        public Navigation2D minusEquals (Navigation2D other) {
            return new Navigation2D(this.x - other.x,
                                    this.y - other.y,
                                 this.theta - other.theta);
        }

        public void addRelativeDeltaToAbsolute(Navigation2D deltaRelativeNav) {
            double PHASE_ROTATION = 0;

            double absoluteX = this.x;
            double absoluteY = this.y;
            double absoluteTheta = this.theta;

            absoluteX +=
                    + deltaRelativeNav.x * Math.cos(absoluteTheta + PHASE_ROTATION)
                    - deltaRelativeNav.y * Math.sin(absoluteTheta + PHASE_ROTATION);
            absoluteY +=
                    + deltaRelativeNav.x * Math.sin(absoluteTheta + PHASE_ROTATION)
                    + deltaRelativeNav.y * Math.cos(absoluteTheta + PHASE_ROTATION);
            absoluteTheta += deltaRelativeNav.theta;

            this.x = absoluteX;
            this.y = absoluteY;
            this.theta = absoluteTheta;
        }
    }


    /**
     * Defines navigation relevant qualities of a Mecanum drive train.
     */
    public static class DriveTrainMecanum{
        public double wheelbaseLength;
        public double wheelbaseWidth;
        public double wheelDiameter;
        public int encoderTicksPerRotation;

        public DriveTrainMecanum(double wheelbaseLength, double wheelbaseWidth,
                                 double wheelDiameter, int encoderTicksPerRotation) {
            this.wheelbaseLength = wheelbaseLength;
            this.wheelbaseWidth = wheelbaseWidth;
            this.wheelDiameter = wheelDiameter;
            this.encoderTicksPerRotation = encoderTicksPerRotation;
        }

        public double getK() {
            return Math.abs(wheelbaseLength/2) + Math.abs(wheelbaseWidth/2);
        }

        public double ticksToRadians(int deltaTicks) {
            return  2 * Math.PI * deltaTicks / encoderTicksPerRotation;
        }
    }

    /**
     * Track encoder values of each Mecanum wheel.
     */
    public static class WheelTicks implements Cloneable
    {
        public int frontLeft;
        public int frontRight;
        public int backLeft;
        public int backRight;

        public WheelTicks(int frontLeft, int frontRight, int backLeft, int backRight) {
            this.frontLeft = frontLeft;
            this.frontRight = frontRight;
            this.backLeft = backLeft;
            this.backRight = backRight;
        }

        public WheelTicks getDeltaFromPrevious(WheelTicks previous) {
            return new WheelTicks(this.frontLeft - previous.frontLeft,
                                this.frontRight -  previous.frontRight,
                                this.backLeft - previous.backLeft,
                                this.backRight - previous.backRight);
        }

        @Override
        public Object clone()
        {
            try {
                return super.clone();
            } catch (CloneNotSupportedException e) {
                e.printStackTrace();
            }
            return null;
        }
    }
}
