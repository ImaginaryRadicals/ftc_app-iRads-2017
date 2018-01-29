package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Utilities.Controller;

import java.util.ResourceBundle;

/**
 * Created by Ashley on 1/27/2018.
 * Opmode for testing various arm control methods.
 * The motivation for this is the new paralelogram arm and enlarged claw,
 * along with higher gearing, which causes the arm to fall under its own weight,
 * which makes the robot very hard to control.
 * Solutions are under investigation which will maintain stability despite the weight.
 */

@TeleOp(name="arm control test", group="testing")
public class ArmControlTest extends OpMode {
    /* Multiple control modes to test
        1) Standard power control using encoders
        2) Move to position using encoders
        3) Custom proportional control
        4) Custom PID controls
     */

    enum ControlMode {
        STANDARD,
        DRIVE_TO_POSITION,
        PROPORTIONAL_CONTROL,
    }

    DcMotor armMotor;
    Controller controller;
    ControlMode controlMode = ControlMode.STANDARD;
    ArmRoutine armControlRoutine;
    double lastLoopTimestamp = 0;
    double initialEncoderTicks;

    private enum ArmState {
        LEVEL_1,
        LEVEL_2,
        LEVEL_3,
        LEVEL_4,
        MANUAL_HOLD,
    }



    @Override
    public void init() {
        controller = new Controller(gamepad1);
        armMotor = hardwareMap.get(DcMotor.class , "ARM_MOTOR");
    }

    @Override
    public void init_loop() {
        controller.update();
        // Select control mode with dpad left and right.
        telemetry.addData("Select Control Mode:","");
        telemetry.addData("Control Mode:", controlMode.toString());
        if(controller.dpadRightOnce()) {
            if( controlMode.ordinal() >=  ControlMode.values().length -1) {
                // If at max value, loop to first.
                controlMode = ControlMode.values()[0];
            } else {
                // Go to next control mode
                controlMode = ControlMode.values()[controlMode.ordinal() + 1];
            }
        } else if ( controller.dpadLeftOnce()) {
            if (controlMode.ordinal() <= 0) {
                // If at first value, loop to last value
                controlMode = ControlMode.values()[ControlMode.values().length-1];
            } else {
                // Go to previous control mode
                controlMode = ControlMode.values()[controlMode.ordinal() - 1];
            }
        }

    }

    @Override
    public void start() {
        initialEncoderTicks = armMotor.getCurrentPosition();
        // Apply settings for selected control mode
        if (controlMode == ControlMode.STANDARD) {
            armControlRoutine = new StandardControl(this);
        } else if (controlMode == ControlMode.DRIVE_TO_POSITION) {
            armControlRoutine = new MoveToPositionControl(this);
        } else if (controlMode == ControlMode.PROPORTIONAL_CONTROL) {

        }
        armControlRoutine.setup();
    }

    @Override
    public void loop() {
        controller.update();
        telemetry.addData("Control Mode:", controlMode.toString());
        telemetry.addData("Arm Encoder", armMotor.getCurrentPosition());

        armControlRoutine.loop();
    }


    // Abstract Arm Routine
    private abstract class ArmRoutine {
        ArmControlTest opMode;
        public abstract void setup();
        public abstract void loop();
    }


    /**
     * Standard control.
     * Moves too fast up, way too fast down, and falls under own weight.
     */
    class StandardControl extends ArmRoutine {

        StandardControl(ArmControlTest opMode) {
            this.opMode = opMode;
        }

        public void setup() {
            armMotor.setDirection(DcMotorSimple.Direction.FORWARD);
            armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        public void loop() {
            armMotor.setPower(analogArmCommand());
        }
    }


    /**
     * Moves slowly but with some jerk. Motor stays active to fight gravity.
     */
    class MoveToPositionControl extends ArmRoutine {
        ArmState armState = ArmState.LEVEL_1;

        MoveToPositionControl(ArmControlTest opMode) {
            this.opMode = opMode;
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
            maxTickRate *= 3;

            double armPower = analogArmCommand();
            // Turn arm power into a target position.
            // Target position is relative to current position, not current target.
            double stepsToCommand = (armPower * maxTickRate * loopPeriod);
            armMotor.setTargetPosition(armMotor.getCurrentPosition() + (int)stepsToCommand);
            armMotor.setPower( Math.abs(armPower+0.1));
        }

        double armStateMachine() {
            double targetEncoderPosition = opMode.armMotor.getTargetPosition();

            if(armState == ArmState.LEVEL_1 || armState == ArmState.LEVEL_2 ||
                    armState == ArmState.LEVEL_3 || armState == ArmState.LEVEL_4) {
                // Goto Position
                targetEncoderPosition = getTicksForPosition(armState.ordinal());
                // Next State logic
                if(controller.dpadUpOnce()) {
                    incrementState();
                } else if (controller.dpadDownOnce()) {
                    decrementState();
                }
            } else if (armState == ArmState.MANUAL_HOLD) {
                // Don't goto a position. Continue to hold position

                // Next State logic
                if (controller.dpadUpOnce()) {
                    incrementState(opMode.armMotor.getCurrentPosition());
                } else if (controller.dpadDownOnce()) {
                    decrementState(opMode.armMotor.getCurrentPosition());
                }
            }
            return targetEncoderPosition;
        }

        void incrementState() {
            int maxStateIndex = ArmState.values().length - 1;
            if (armState.ordinal() < maxStateIndex) {
                armState = ArmState.values()[armState.ordinal() + 1];
            }
        }

        void incrementState(int currentEncoderValues) {

        }

        void decrementState() {
            if (armState.ordinal() > 0) {
                armState = ArmState.values()[armState.ordinal() - 1];
            }
        }

        void decrementState(int currentEncoderValues) {

        }

    }



    private double analogArmCommand() {
        double left_trigger = controller.left_trigger;
        double right_trigger = controller.right_trigger;
        double armPower = 0;

        if (left_trigger > 0.1) {
            armPower = -(0.05 + 0.45 * left_trigger);
        } else if (right_trigger > 0.1) {
            armPower = (0.05 + 0.45 * right_trigger);
        } else {
            armPower = 0;
        }
        return armPower;
    }

    double getTicksForPosition(int glyphLevel) {
        // 4 Glyph levels, 0-3

        double leverArm = 17; // inches

        double height = 1+ glyphLevel * 6.0; // Block size
        //double angleRequired = Math.cos

        return initialEncoderTicks + (double) glyphLevel;
    }
}
