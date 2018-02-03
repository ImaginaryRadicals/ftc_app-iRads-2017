package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Utilities.AutoDrive;
import org.firstinspires.ftc.teamcode.Utilities.Color;
import org.firstinspires.ftc.teamcode.Utilities.Constants;
import org.firstinspires.ftc.teamcode.Utilities.Controller;
import org.firstinspires.ftc.teamcode.Utilities.Mecanum;
import org.firstinspires.ftc.teamcode.Utilities.MecanumNavigation;

/**
 * Created by ryderswan on 12/5/17.
 */


@TeleOp (name="Manual Mecanum", group="Manual")
//@Disabled
public class Manual extends RobotHardware {

    public Controller controller = null;
    public Controller copilotController = null;
    public boolean copilotControllerActive = false;
    public MecanumNavigation mecanumNavigation;
    public AutoDrive autoDrive;
    private boolean use_telemetry = true;
    private boolean forward_drive = true;
    private boolean exponential_input = true;
    private boolean slow_mode = false;
    private ClawState clawState = ClawState.CLAW_STOWED;
    private boolean fastMode = true;
    private MecanumNavigation.Navigation2D pickupPosition = new MecanumNavigation.Navigation2D(0,0,0);

    // Variables for the claw states.
    private enum ClawState {
        CLAW_STOWED, CLAW_OPEN, CLAW_RELEASE, CLAW_CLOSED, CLAW_TESTING,
    }

    // Selectable during init_loop()
    private enum ArmControlMode {
        STANDARD,
        NO_ENCODER_ARM_CONTROL,
        MOVE_TO_POSITION,
    }
    private ArmControlMode armControlMode = ArmControlMode.MOVE_TO_POSITION;
    private ArmRoutine armRoutine; // Object which initializes and runs arm controls.

    int initialEncoderTicks = 0;
    private enum ArmState {
        LEVEL_1,
        LEVEL_2,
        LEVEL_3,
        LEVEL_4,
        MANUAL,
    }
    private ArmState armState = ArmState.MANUAL;


    @Override
    public void init() {
        super.init();
        controller = new Controller(gamepad1);
        copilotController = new Controller(gamepad2);
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
    }

    @Override
    public void init_loop() {
        controller.update();
        // Select control mode with dpad left and right.
        telemetry.addData("Select Mode"," using Dpad left/right");
        telemetry.addData("Arm Control Mode:", armControlMode.toString());
        armControlMenuEnumUpdate();
    }

    @Override
    public void start() {
        super.start();
        initialEncoderTicks = getEncoderValue(MotorName.ARM_MOTOR);
        // Initialize Arm Controls
        if (armControlMode == ArmControlMode.STANDARD) {
            armRoutine = new StandardControl(this);
        } else if (armControlMode == ArmControlMode.MOVE_TO_POSITION) {
            armRoutine = new MoveToPositionControl(this);
        } else if (armControlMode == ArmControlMode.NO_ENCODER_ARM_CONTROL) {
            armRoutine = new NoEncoderArmControl(this);
        }

        armRoutine.setup(); // Run init specific to arm routine.
        storeClaw();
        armServoTop();
    }

    @Override
    public void loop() {
        // Keep timers updated
        super.loop();
        // Keep controller rising edge trigger updated.
        controller.update();
        copilotController.update();
        // Update mecanum encoder navigation via opMode context.
        mecanumNavigation.update();


        // Chord Commands
        if (controller.leftBumper() && controller.rightBumper()) {

            if(!controller.B() && !controller.X()) {
                stopAllMotors(); // Safety first! (Otherwise motors can run uncontrolled.)
            }
            jewelArmTesting();
            clawTestControls();

            // Reset navigation position to zero.
            if (controller.YOnce() || controller.dpadUpOnce()) {
                mecanumNavigation.setCurrentPosition(new MecanumNavigation.Navigation2D(0, 0, 0));
            }
            if (controller.dpadDownOnce()) { // Set current position to pickup position
                pickupPosition = (MecanumNavigation.Navigation2D) mecanumNavigation.currentPosition.clone();
            }
            if (controller.dpadLeftOnce()) {
                mecanumNavigation.setCurrentPosition(new MecanumNavigation.Navigation2D(0,6.5,0));
            }
            if (controller.dpadRightOnce()) {
                mecanumNavigation.setCurrentPosition(new MecanumNavigation.Navigation2D(0,-6.5,0));
            }
            // Test autoDrive
            if (controller.X()) {
                autoDrive.driveToPosition(new MecanumNavigation.Navigation2D(0,0,0), 1);
            }
            // Test autoDrive
            if (controller.B()) {  // Removed 'once' trigger.
                autoDrive.driveToPosition(pickupPosition, 1);
            }
            // Full power mode
            if (controller.AOnce()) {
                fastMode = !fastMode;
            }
        } else {
            // Non-chord robot inputs

            // Toggle copilot controller activation
            if (copilotController.startOnce() || controller.startOnce()) {
                copilotControllerActive = ! copilotControllerActive;
            }

            robotControls();
        }


        if (use_telemetry) {
            telemetry.addData("CoPilot Active", copilotControllerActive);
            telemetry.addLine();
            telemetry.addData("Slow", slow_mode);
            telemetry.addData("Fast", fastMode);
            telemetry.addData("Forward Drive", forward_drive);
            telemetry.addData("Arm Encoder", getEncoderValue(MotorName.ARM_MOTOR));
            telemetry.addData("pickupPosition",pickupPosition.toString());
            telemetry.addLine(); // Visual Space
            mecanumNavigation.displayPosition();
            telemetry.addLine(); // Visual Space
        }
    }

    private void robotControls() {

        //Drive Motor control
        if (controller.leftStickButtonOnce()) {
            forward_drive = !forward_drive;
        }

        // Slow mode controls
        if (controller.rightStickButtonOnce()) {
            slow_mode = !slow_mode;
        }

        double sign = forward_drive ? 1 : -1;
        double max_rate = 0.8;
        if (slow_mode) {
            max_rate = 0.3;
        } else if (fastMode) {
            max_rate = 1.0;
        }
        double exponential = exponential_input ? 3 : 1;
        // Manual control with sticks, auto drive with dpad.
        if (!controller.dpadUp() && !controller.dpadDown() && !controller.dpadLeft() && !controller.dpadRight()) {
            setDriveForSimpleMecanum(
                    sign * max_rate * Math.pow(gamepad1.left_stick_x, exponential),
                    sign * max_rate * Math.pow(gamepad1.left_stick_y, exponential),
                    max_rate * Math.pow(gamepad1.right_stick_x, exponential),
                    max_rate * Math.pow(gamepad1.right_stick_y, exponential));
        } else {
            // Only use dpad if in standard arm control mode.
            if (armRoutine.getClass() == StandardControl.class) {
                // Auto driving controls on dpad.
                if (controller.dpadUp()) {
                    autoDrive.rotateThenDriveToPosition(new MecanumNavigation.Navigation2D(0, 0, 0), 1);
                }
                if (controller.dpadLeft()) {
                    autoDrive.rotateThenDriveToPosition(new MecanumNavigation.Navigation2D(0, 7.63, 0), 1);
                }
                if (controller.dpadRight()) {
                    autoDrive.rotateThenDriveToPosition(new MecanumNavigation.Navigation2D(0, -7.63, 0), 1);
                }
                if (controller.dpadDown()) {
                    autoDrive.rotateThenDriveToPosition((MecanumNavigation.Navigation2D) pickupPosition.clone(), 1);
                }
            }
        }

        // Arm controls
        armRoutine.loop();

        // Claw Controls
        clawStateMachine();

    }



    // Claw Control
    // Right closes and Left Opens
    private void clawStateMachine() {
        switch (clawState)
        {
            case CLAW_STOWED:
                storeClaw();
                if (controller.rightBumperOnce() || copilotControllerActive && copilotController.rightBumperOnce()) {
                    clawState = ClawState.CLAW_OPEN;
                }
                break;
            case CLAW_OPEN:
                openClaw();
                if (controller.rightBumperOnce() || copilotControllerActive && copilotController.rightBumperOnce()) {
                    clawState = ClawState.CLAW_CLOSED;
                } else if (controller.leftBumperOnce() || copilotControllerActive && copilotController.leftBumperOnce()) {
                    clawState = ClawState.CLAW_STOWED;
                }
                break;
            case CLAW_CLOSED:
                closeClaw();
                if (controller.leftBumperOnce() || copilotControllerActive && copilotController.leftBumperOnce()) {
                    clawState = ClawState.CLAW_RELEASE;
                }
                break;
            case CLAW_RELEASE:
                slightOpenClaw();
                if (controller.rightBumperOnce() || copilotControllerActive && copilotController.rightBumperOnce()) {
                    clawState = ClawState.CLAW_CLOSED;
                } else if (controller.leftBumperOnce() || copilotControllerActive && copilotController.leftBumperOnce()) {
                    clawState = ClawState.CLAW_OPEN;
                }
                break;
            case CLAW_TESTING:
                // No position commanded, leave where it is.
                if (controller.rightBumperOnce() || copilotControllerActive && copilotController.rightBumperOnce()) {
                    clawState = ClawState.CLAW_CLOSED;
                } else if (controller.leftBumperOnce() || copilotControllerActive && copilotController.leftBumperOnce()) {
                    clawState = ClawState.CLAW_OPEN;
                }
                break;
            default:
                break;
        }
    }

    private void clawTestControls() {
        double clawRate = 1;
        // If (and only if) the claw is moved in testing, set state to leave claw where it is.
        if ( controller.left_stick_y != 0 ) {
            clawState = ClawState.CLAW_TESTING;
        }
        double clawTarget = controller.left_stick_y * clawRate * getLastPeriodSec() + getAngle(ServoName.CLAW_LEFT);
        clawTarget = Range.clip(clawTarget, 0, 1);
        setPositionClaw(clawTarget);
    }

    private void jewelArmTesting() {
        double servoRate = 1;
        double stepSize = servoRate * getLastPeriodSec() * -controller.right_stick_y;
        double jewelServoTargetPosition = Range.clip(stepSize + getAngle(ServoName.JEWEL_ARM), 0, 1);
        setAngle(ServoName.JEWEL_ARM, jewelServoTargetPosition);
    }

    private double getAnalogArmCommand(Controller thisController) {
        double left_trigger = thisController.left_trigger;
        double right_trigger = thisController.right_trigger;
        double threshold = 0.05;

        if (left_trigger > threshold) {
            return  - 0.5 * left_trigger;
        } else if (right_trigger > threshold) {
            return 0.5 * right_trigger;
        } else {
            return 0.0;
        }
    }


    private void armControlMenuEnumUpdate() {
        // Select control mode with dpad left and right.
        if(controller.dpadRightOnce()) {
            if( armControlMode.ordinal() >=  armControlMode.values().length -1) {
                // If at max value, loop to first.
                armControlMode = ArmControlMode.values()[0];
            } else {
                // Go to next control mode
                armControlMode = ArmControlMode.values()[armControlMode.ordinal() + 1];
            }
        } else if ( controller.dpadLeftOnce()) {
            if (armControlMode.ordinal() <= 0) {
                // If at first value, loop to last value
                armControlMode = ArmControlMode.values()[armControlMode.values().length-1];
            } else {
                // Go to previous control mode
                armControlMode = ArmControlMode.values()[armControlMode.ordinal() - 1];
            }
        }
    }



    // Abstract routine for controlling the arm.
    private abstract class ArmRoutine {
        Manual opMode;
        DcMotor armMotor;
        public ArmRoutine(Manual opMode) {
            this.opMode = opMode;
            this.armMotor = opMode.hardwareMap.get(DcMotor.class, "ARM_MOTOR");
        }
        public abstract void setup();
        public abstract void loop();
    }

    /**
     * Standard control.
     * Moves too fast up, way too fast down, and falls under own weight.
     */
    class StandardControl extends ArmRoutine {
        StandardControl(Manual opMode) {
            super(opMode);
        }

        public void setup() {
            armMotor.setDirection(DcMotorSimple.Direction.FORWARD);
            armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        public void loop() {
            double pilotArmPower = getAnalogArmCommand(opMode.controller);
            double copilotArmPower = copilotControllerActive ? getAnalogArmCommand(copilotController) : 0;
            armMotor.setPower(pilotArmPower + copilotArmPower);
        }
    }


    /**
     * Standard control, but without using encoder.
     */
    class NoEncoderArmControl extends ArmRoutine {
        NoEncoderArmControl(Manual opMode) {
            super(opMode);
        }

        public void setup() {
            armMotor.setDirection(DcMotorSimple.Direction.FORWARD);
            armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        public void loop() {
            double pilotArmPower = getAnalogArmCommand(opMode.controller);
            double copilotArmPower = copilotControllerActive ? getAnalogArmCommand(copilotController) : 0;
            double totalArmPower = pilotArmPower + copilotArmPower;
            if (totalArmPower < 0) {
                totalArmPower *= 0.5; // Reduce downward power by half (when without encoders)
            }
            armMotor.setPower(pilotArmPower + copilotArmPower);
        }
    }


    /**
     * Use move to position mode to control the arm.
     */
    class MoveToPositionControl extends ArmRoutine {
        private double lastLoopTimestamp = 0;
        private double previousAnalogInput = 0;
        private double currentAnalogInput = 0;
        private boolean armLiftOffset = false;
        private double dPadOffsetOverride = -1;
        private ClawState previousClawState = ClawState.CLAW_STOWED;
        private ArmState previousArmState = ArmState.MANUAL;
        private ArmGeometry armGeometry;
        private RobotStateSnapshot clawLastOpen = new RobotStateSnapshot(time,
                new MecanumNavigation.Navigation2D(0,0,0),
                ArmState.MANUAL);
        private RobotStateSnapshot clawLastClosed = new RobotStateSnapshot(time,
                new MecanumNavigation.Navigation2D(0,0,0),
                ArmState.MANUAL);

        MoveToPositionControl(Manual opMode) {
            super(opMode);
        }

        public void setup() {
            armMotor.setDirection(DcMotorSimple.Direction.FORWARD);
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            armMotor.setTargetPosition(initialEncoderTicks);
            armGeometry = new ArmGeometry(initialEncoderTicks); // Set assumed bottom position.
        }

        public void loop() {
            double loopPeriod = time - lastLoopTimestamp;
            lastLoopTimestamp = time;
            telemetry.addData("Target Distance", armMotor.getTargetPosition() - armMotor.getCurrentPosition());
            telemetry();

            // MaxTick rate sec = pulses per rotation * no load speed RPM / 60
            double maxTickRate = 1680 * 105 / 60;
            // Fudge Factor
            maxTickRate *= 6;

            double pilotArmPower = getAnalogArmCommand(controller);
            double copilotArmPower = copilotControllerActive ? getAnalogArmCommand(copilotController) : 0;
            double armPower = pilotArmPower + copilotArmPower;
            updateAnalogInputReleasedTrigger(armPower);
            // Turn arm power into a target position.
            // Target position is relative to current position, not current target.


            if (Math.abs(armPower) >= 0.05) {
                double stepsToCommand = (armPower * maxTickRate * loopPeriod);
                armMotor.setTargetPosition(armMotor.getCurrentPosition() + (int)stepsToCommand);
                armMotor.setPower( Math.abs(armPower));
                armState = ArmState.MANUAL; // Disable state machine until dpad inputs.
            } else {
                if(analogInputReleasedOnce()){
                    // Once, when arm input is first released, set target position to current position.
                    armMotor.setTargetPosition(armMotor.getCurrentPosition());
                }
                // When Analog input == 0, hold (or moveTo) targetPosition(), set either
                // by the position analog inputs ceased, or by the armStateMachine().
                int armError = armMotor.getCurrentPosition() - armMotor.getTargetPosition();
                armMotor.setPower(getArmPowerFromError(armError));
            }

            // Use dpad left/right to disable/enable the level offset lift, which lifts the arm
            // a small bit (1 inch) to clear other blocks when placing them.
            if (controller.dpadRightOnce() || copilotControllerActive && copilotController.dpadRightOnce()) {
                // Activate offset, lift by ~ 1 inch over state level.
                dPadOffsetOverride = 1;
                armLiftOffset = true;
            } else if (controller.dpadLeftOnce() || copilotControllerActive && copilotController.dpadLeftOnce()) {
                dPadOffsetOverride = 0;
                armLiftOffset = false;
            }

            armOffsetStateMachine();
            // Run Arm State machine.  Does nothing if in Manual state (after analog inputs)
            armStateMachine(controller);
            if (copilotControllerActive) {
                armStateMachine(copilotController);
            }
        }

        /**
         * Negative error means our current position is below the desired position.
         * Error = current - target
         * @param errorTicks
         * @return
         */
        private double getArmPowerFromError(int errorTicks) {
            //double powerScale = AutoDrive.rampDown(errorTicks,300, 1, 0.2);
            //return 0.3 * powerScale;
            if (errorTicks <= 0) {
                return 0.18;
            } else {
                return 0.15;
            }
        }


        private boolean analogInputReleasedOnce() {
            return (previousAnalogInput != 0 && currentAnalogInput == 0);
        }

        private void updateAnalogInputReleasedTrigger(double analogInput) {
            previousAnalogInput = currentAnalogInput;
            currentAnalogInput = analogInput;
        }

        public void telemetry() {
            telemetry.addLine();
            telemetry.addData("Offset Ticks:", armGeometry.offsetTicks);
            telemetry.addData("Arm State:", armState);
            telemetry.addLine();
            telemetry.addData("Arm Height Inches", armGeometry.getHeightInchesFromTicks(armMotor.getCurrentPosition()));
            telemetry.addData("Targete Angle Degrees", armGeometry.getAngleTargetDegreesFromTicks(armMotor.getCurrentPosition()));
            telemetry.addData("ticks per degree", armGeometry.ticksPerDegree);
            telemetry.addLine();
            telemetry.addData("lift offset active:", armLiftOffset);
            telemetry.addData("dPad offset Override", dPadOffsetOverride);
        }


        private void armStateMachine(Controller controller) {
            double levelSpacingInches = Constants.BLOCK_HEIGHT_INCHES;
            double levelOffsetInches = armLiftOffset ? Constants.BLOCK_OFFSET_INCHES : 0;

            ArmState nextState = armState; // Default setting
            switch (armState)
            {
                case LEVEL_1:
                    armMotor.setTargetPosition(armGeometry.getTicksFromHeightInches(0*levelSpacingInches + levelOffsetInches));
                    // Next State Logic
                    if (controller.dpadUpOnce()) {
                        nextState = ArmState.LEVEL_2;
                    }
                    break;
                case LEVEL_2:
                    armMotor.setTargetPosition(armGeometry.getTicksFromHeightInches(1*levelSpacingInches + levelOffsetInches));
                    // Next State Logic
                    if (controller.dpadDownOnce()) {
                        nextState = ArmState.LEVEL_1;
                    } else if (controller.dpadUpOnce()) {
                        nextState = ArmState.LEVEL_3;
                    }
                    break;
                case LEVEL_3:
                    armMotor.setTargetPosition(armGeometry.getTicksFromHeightInches(2*levelSpacingInches + levelOffsetInches));
                    // Next State Logic
                    if (controller.dpadDownOnce()) {
                        nextState = ArmState.LEVEL_2;
                    } else if (controller.dpadUpOnce()) {
                        nextState = ArmState.LEVEL_4;
                    }
                    break;
                case LEVEL_4:
                    armMotor.setTargetPosition(armGeometry.getTicksFromHeightInches(3*levelSpacingInches + levelOffsetInches));
                    // Next State Logic
                    if (controller.dpadDownOnce()) {
                        nextState = ArmState.LEVEL_3;
                    }
                    break;
                case MANUAL:
                    // Do nothing. Current targetPosition should be unmodified by this function.
                    // Next State Logic, move to nearest state to current position.
                    double heightInches = armGeometry.getHeightInchesFromTicks(armMotor.getCurrentPosition());
                    if (controller.dpadDownOnce()) {
                        if (heightInches < 1*levelSpacingInches) {
                            nextState = ArmState.LEVEL_1;
                        } else if (heightInches < 2*levelSpacingInches) {
                            nextState = ArmState.LEVEL_2;
                        } else if (heightInches < 3*levelSpacingInches) {
                            nextState = ArmState.LEVEL_3;
                        } else {
                            nextState = ArmState.LEVEL_4;
                        }
                    } else if (controller.dpadUpOnce()) {
                        if (heightInches > 2*levelSpacingInches) {
                            nextState = ArmState.LEVEL_4;
                        } else if (heightInches > 1*levelSpacingInches) {
                            nextState = ArmState.LEVEL_3;
                        } else if (heightInches > 0*levelSpacingInches) {
                            nextState = ArmState.LEVEL_2;
                        } else {
                            nextState = ArmState.LEVEL_1;
                        }
                    }
                    break;
                default:
                    nextState = ArmState.MANUAL;
                    break;
            }
            if (previousArmState != armState) {
                // Arm state shifted, initialize
                dPadOffsetOverride = -1;
            }
            previousArmState = armState; // prepare for next loop
            armState = nextState;
        }

        // Store robot state snapshot. Used by armOffsetStateMachine()
        class RobotStateSnapshot {
            public double time;
            public MecanumNavigation.Navigation2D position;
            public ArmState armState;

            RobotStateSnapshot(double time, MecanumNavigation.Navigation2D position, ArmState armState) {
                this.time = time;
                this.position = position;
                this.armState = armState;
            }
        }

        /**
         * State machine to determine whether to automatically toggle offset on or off.
         * Updates and uses the value of previousArmState.
         * Detects transitions to and from CLAW_CLOSED.
         * On transitions to CLAW_CLOSED, updates clawLastOpen[time, navPosition, armState]
         * On transitions from CLAW_CLOSED, updates clawLastClosed[time, navPosition, armState]
         * (then updates previousClawState)
         * Then, based on current claw state (Closed or !Closed) and the difference between
         * time, location, and armState, the appropriate offset setting is determined.
         */
        private void armOffsetStateMachine() {
            double timeDelay = 2;
            double distanceDelay = 1.5;
            double angleDegreesDelay = 8;


            if (clawState == ClawState.CLAW_CLOSED && previousClawState != ClawState.CLAW_CLOSED) {
                // Detect transitions to CLAW_CLOSED, and store most recent clawLastOpen telemetry
                clawLastOpen = new RobotStateSnapshot(time,
                        (MecanumNavigation.Navigation2D) mecanumNavigation.currentPosition.clone(),
                        armState);
            } else if (clawState != ClawState.CLAW_CLOSED && previousClawState == ClawState.CLAW_CLOSED) {
                // Detect transitions from CLAW_CLOSED, and store most recent clawLastClosed telemetry
                clawLastClosed = new RobotStateSnapshot(time,
                        (MecanumNavigation.Navigation2D) mecanumNavigation.currentPosition.clone(),
                        armState);
            }
            previousClawState = clawState; // Updated for next loop.

            // Give dpad precedence over autonomous decisions.
            if (dPadOffsetOverride == 1) {
                armLiftOffset = true;
            } else if (dPadOffsetOverride == 0) {
                armLiftOffset = false;

            } else if (clawState == ClawState.CLAW_CLOSED) {
                // Claw is closed
                if (armState != clawLastOpen.armState ||
                        (time - clawLastOpen.time) > timeDelay ||
                        mecanumNavigation.currentPosition.distanceTo(clawLastOpen.position) > distanceDelay ||
                        Math.abs(mecanumNavigation.currentPosition.angleDegreesTo(clawLastOpen.position)) > angleDegreesDelay) {
                    armLiftOffset = true;
                }

            } else if (clawState != ClawState.CLAW_CLOSED) {
                // Claw is open
                if (armState != clawLastClosed.armState ||
                        armState == ArmState.LEVEL_1 ||
                        (time - clawLastClosed.time) > timeDelay ||
                        mecanumNavigation.currentPosition.distanceTo(clawLastClosed.position) > distanceDelay ||
                        Math.abs(mecanumNavigation.currentPosition.angleDegreesTo(clawLastClosed.position)) > angleDegreesDelay) {
                    armLiftOffset = false;
                }
            }
        }

    }


    /** Positions are measured after turning on the robot with the arm
     * at its lowest position, but with the arm gearing taught and ready
     * to lift.  Bottom angle is 0 by definition, but offset ticks are possible,
     * which shift every tick positoin up.
     * Note: bottomAngle is expected to be a negative number.
     */
    class ArmGeometry {
        private int offsetTicks = 0;
        private double armLengthInches, bottomAngleDegrees, topAngleDegrees;
        private int bottomTicks, levelTicks, topTicks;
        private double ticksPerDegree, heightLevelInches;
        private double slopDegrees, slopTicks;

        ArmGeometry() {
            this(0);
        }

        ArmGeometry(int ticksOffset) {
            this(Constants.ARM_LENGTH_INCHES, Constants.ARM_BOTTOM_ANGLE_DEGREES,
                    Constants.ARM_TOP_ANGLE_DEGREES, Constants.ARM_BOTTOM_TICKS,
                    Constants.ARM_LEVEL_TICKS, Constants.ARM_TOP_TICKS, ticksOffset);
        }

        ArmGeometry(double lengthInches, double bottomAngleDegrees, double topAngleDegrees,
                    int bottomTicks, int levelTicks, int topTicks, int ticksOffset) {
            this.armLengthInches = lengthInches;
            this.bottomAngleDegrees = bottomAngleDegrees;
            this.topAngleDegrees = topAngleDegrees;
            this.bottomTicks = bottomTicks;
            this.levelTicks = levelTicks;
            this.topTicks = topTicks;
            alignOffsets(ticksOffset);
        }

        void alignOffsets(int tickOffset) {
            bottomTicks += tickOffset;
            levelTicks += tickOffset;
            topTicks += tickOffset;
            offsetTicks += tickOffset;
            ticksPerDegree = (topTicks - bottomTicks) / (topAngleDegrees - bottomAngleDegrees);
            heightLevelInches = - armLengthInches * Math.sin(bottomAngleDegrees*Math.PI/180);
        }

        /**
         * height is taken to be zero when the claw is at
         * bottomTicks, which is 0 ticks unless an offset is added,
         * and bottomAngleDegrees.
         * @param heightInches
         * @return int encoder ticks
         */
        public int getTicksFromHeightInches(double heightInches) {
            // Calculate angle above level for given height
            double angleDeg = Math.asin((heightInches - heightLevelInches)/armLengthInches)*(180/Math.PI);
            return (int) ( (angleDeg - bottomAngleDegrees)* ticksPerDegree) + offsetTicks;
        }

        public double getHeightInchesFromTicks(int armTicks) {
            // Convert ticks to degrees above level
            double angleDeg = (armTicks - offsetTicks) / ticksPerDegree;
            return heightLevelInches + armLengthInches * Math.sin((angleDeg + bottomAngleDegrees)* (Math.PI/180));
        }

        public double getAngleTargetDegreesFromTicks(int armTicks) {
            double angleDeg = (armTicks - offsetTicks) / ticksPerDegree + bottomAngleDegrees;
            return angleDeg;
        }
    }


}