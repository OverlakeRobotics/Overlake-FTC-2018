package org.firstinspires.ftc.teamcode.systems.arm;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.components.scale.LogarithmicRamp;
import org.firstinspires.ftc.teamcode.components.scale.Point;
import org.firstinspires.ftc.teamcode.components.scale.Ramp;
import org.firstinspires.ftc.teamcode.opmodes.IBaseOpMode;
import org.firstinspires.ftc.teamcode.systems.System;

/**
 * Arm System rotates the arm up and down through angular motion. The arm moves using a potentiometer
 * and ramps as it approaches the top of the robot and bottom of the robot to prevent damage to the
 * arm
 */
public class ArmSystem extends System {
    private DcMotor motor1;
    private DcMotor motor2;
    private Servo armRelease;
    private AnalogInput potentiometer;
    private Ramp rampUp;
    private Ramp rampDown;
    private Ramp fallingRamp;

    private final double MaximumVoltage = 2.3;
    private final double MinimumVoltage = 0.75;
    private final double LatchVoltage = 0.98;
    public final double LatchPreparationVoltage = 1.7;
    private final double LatchPreparationMargin = 0.5;
    private final double WheelVoltage = 1.4 ;
    private final double MaxPower = 0.5;
    private final double LatchPower = 1;
    private final double MinPower = 0.15;
    private boolean isRamping;

    private ArmState currentState;

    /**
     * Builds a new Arm System for the given opmode
     * @param opMode the opmode that the arm system is currently running on
     */
    public ArmSystem(IBaseOpMode opMode) {
        super(opMode, "ArmSystem");
        motor1 = hardwareMap.dcMotor.get("parallelM1");
        motor2 = hardwareMap.dcMotor.get("parallelM2");
        armRelease = hardwareMap.servo.get("armRelease");
        potentiometer = hardwareMap.get(AnalogInput.class, "potentiometer");
        isRamping = true;
        setState(ArmState.IDLE);
        rampUp = new LogarithmicRamp(
            new Point(MaximumVoltage / 2, MaxPower),
            new Point(MaximumVoltage, MinPower)
        );
        rampDown = new LogarithmicRamp(
            new Point(MinimumVoltage, MinPower),
            new Point(MaximumVoltage / 2, MaxPower)
        );
    }

    /**
     * Sets the state of the arm
     * @param state new state of the arm
     */
    public void setState(ArmState state) {
        currentState = state;
    }

    /**
     * Runs the arm
     */
    public void run() {
        log.info("Arm Power", isRamping);
        log.info("Arm Power", potentiometer.getVoltage());
        switch (currentState) {
            case RELEASE_PIN:
                releaseArmPin();
                break;
            case ROTATING_TOP:
                rotateTop();
                break;
            case ROTATING_BOTTOM:
                rotateBottom();
                break;
            default:
                stop();
                break;
        }
    }

    /**
     * Rotates the arm towards the bottom
     */
    public void rotateBottom() {
        motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        if (!isAtBottom()) {
            runMotors(ArmDirection.DOWN);
        } else {
            stop();
        }
    }

    /**
     * Checks if the arm is at the bottom
     * @return Returns true if the arm is at the bottom
     */
    public boolean isAtBottom() {
        return potentiometer.getVoltage() <= MinimumVoltage;
    }

    /**
     * Runs the motors in a given direction
     * @param direction The direction to run the motors
     */
    public void runMotors(ArmDirection direction) {
        if (direction == ArmDirection.DOWN) {
            double power = getPower(rampDown);
            motor1.setPower(power);
            motor2.setPower(-power);
        } else {
            double power = getPower(rampUp);
            motor1.setPower(-power);
            motor2.setPower(power);
        }
    }

    /**
     * Rotates the arm to the top
     */
    public void rotateTop() {
        motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        if (!isAtTopPosition()) {
            runMotors(ArmDirection.UP);
        } else {
            stop();
        }
    }

    /**
     * Checks that the arm is at the top
     * @return Returns true if the arm is at the top
     */
    public boolean isAtTopPosition() {
        return potentiometer.getVoltage() >= MaximumVoltage;
    }

    /**
     * Stops the arm from running
     */
    public void stop() {
        setState(ArmState.IDLE);
        motor1.setPower(0);
        motor2.setPower(0);
    }

    /**
     * toggles the ramping of the robot
     */
    public void toggleRamping() {
        isRamping = !isRamping;
    }

    /**
     * sets the power of the robot
     */
    private double getPower(Ramp ramp) {
        if (isRamping) {
            return ramp.scaleX(potentiometer.getVoltage());
        } else {
            return LatchPower;
        }
    }

    /**
     * Releases the pin of that holds the arm up when latching
     */
    public void releaseArmPin() {
        fallingRamp = new LogarithmicRamp(
                new Point(0.0001, 0.1),
                new Point(potentiometer.getVoltage(), MaxPower)
        );
        armRelease.setPosition(0);
        setState(ArmState.IDLE);
    }

    /**
     * Sets the pin of that holds the arm up when latching
     */
    public void setArmPin() {
        armRelease.setPosition(0.7);
        setState(ArmState.IDLE);
    }

    /**
     * Gets the position of the potentiometer
     * @return the potentiometer voltage
     */
    public ArmDirection getDirectionToRun(double voltage) {
        return voltage >= potentiometer.getVoltage() ?
                ArmDirection.DOWN :
                ArmDirection.UP;
    }

    /**
     * Checks if the arm is prepared to latch
     * @return Returns true if the arm is prepared to latch
     */
    public boolean canLatch() {
        return potentiometer.getVoltage() >= LatchPreparationVoltage - LatchPreparationMargin &&
                potentiometer.getVoltage() <= LatchPreparationVoltage + LatchPreparationMargin;
    }


    /**
     * Checks if the arm is collapsed and latched on the lander
     * @return true if the arm is collapsed and latched on the lander
     */
    public boolean isLatched() {
        return LatchVoltage >= potentiometer.getVoltage();
    }

    public boolean wheelsOnGround() {
        return WheelVoltage <= potentiometer.getVoltage();
    }
}
