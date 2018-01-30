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
    private boolean fastMode = false;
    private MecanumNavigation.Navigation2D pickupPosition = new MecanumNavigation.Navigation2D(0,0,0);

    // Variables for the claw states.
    private enum ClawState {
        CLAW_STOWED, CLAW_OPEN, CLAW_RELEASE, CLAW_CLOSED, CLAW_TESTING,
    }

    // Selectable during init_loop()
    private enum ArmControlMode {
        STANDARD,
        MOVE_TO_POSITION,
    }
    private ArmControlMode armControlMode = ArmControlMode.STANDARD;
    private ArmRoutine armRoutine; // Object which initializes and runs arm controls.

    double initialEncoderTicks;
    private enum ArmState {
        LEVEL_1,
        LEVEL_2,
        LEVEL_3,
        LEVEL_4,
        MANUAL_HOLD,
    }
    private ArmState armState = ArmState.MANUAL_HOLD;


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
     * Use move to position mode to control the arm.
     */
    class MoveToPositionControl extends ArmRoutine {
        private double lastLoopTimestamp = 0;
        private double previousAnalogInput = 0;
        private double currentAnalogInput = 0;

        MoveToPositionControl(Manual opMode) {
            super(opMode);
        }

        public void setup() {
            armMotor.setDirection(DcMotorSimple.Direction.FORWARD);
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        public void loop() {
            double loopPeriod = lastLoopTimestamp - time;
            lastLoopTimestamp = time;
            telemetry.addData("Target Distance", armMotor.getTargetPosition() - armMotor.getCurrentPosition());

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
                armMotor.setPower( Math.abs(armPower+0.1));
            } else {
                if(analogInputReleasedOnce()){
                    // Once, when arm input is first released, set target position to current position.
                    armMotor.setTargetPosition(armMotor.getCurrentPosition());
                }
                armMotor.setPower(0.5); // Station keeping
            }

        }

        private boolean analogInputReleasedOnce() {
            return (previousAnalogInput != 0 && currentAnalogInput == 0);
        }

        private void updateAnalogInputReleasedTrigger(double analogInput) {
            previousAnalogInput = currentAnalogInput;
            currentAnalogInput = analogInput;
        }

    }


}