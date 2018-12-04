package org.firstinspires.ftc.teamcode.systems.arm;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.components.scale.LogarithmicRamp;
import org.firstinspires.ftc.teamcode.components.scale.Point;
import org.firstinspires.ftc.teamcode.components.scale.Ramp;
import org.firstinspires.ftc.teamcode.systems.base.System;

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

    private final double PotentiometerMaximum = 2.3;
    private final double PotentiometerMinimum = 0.8;
    private final double MaxPower = 0.3;
    private final double MinPower = 0.01;

    private ArmState currentState;
    private double latchVoltage;

    /**
     * Builds a new Arm System for the given opmode
     * @param opMode the opmode that the arm system is currently running on
     */
    public ArmSystem(OpMode opMode) {
        super(opMode, "ArmSystem");
        motor1 = hardwareMap.dcMotor.get("parallelM1");
        motor2 = hardwareMap.dcMotor.get("parallelM2");
        armRelease = hardwareMap.servo.get("armRelease");
        potentiometer = hardwareMap.get(AnalogInput.class, "potentiometer");
        setState(ArmState.IDLE);
        rampUp = new LogarithmicRamp(
                new Point(PotentiometerMaximum / 2, MaxPower),
                new Point(PotentiometerMaximum, MinPower)
        );
        rampDown = new LogarithmicRamp(
                new Point(PotentiometerMinimum, MinPower),
                new Point(PotentiometerMaximum / 2, MaxPower)
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
        telemetry.log("voltage", potentiometer.getVoltage());
        telemetry.write();
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
            case FALLING:
                slowFall();
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
            motor1.setPower(rampDown.scaleX(potentiometer.getVoltage()));
            motor2.setPower(-rampDown.scaleX(potentiometer.getVoltage()));
        } else {
            stop();
        }
    }

    /**
     * Checks if the arm is at the bottom
     * @return Returns true if the arm is at the bottom
     */
    private boolean isAtBottom() {
        return potentiometer.getVoltage() <= PotentiometerMinimum;
    }

    /**
     * Rotates the arm to the top
     */
    public void rotateTop() {
        motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        if (!isAtTopPosition()) {
            motor1.setPower(-rampUp.scaleX(potentiometer.getVoltage()));
            motor2.setPower(rampUp.scaleX(potentiometer.getVoltage()));
        } else {
            stop();
        }
    }

    /**
     * Checks that the arm is at the top
     * @return Returns true if the arm is at the top
     */
    private boolean isAtTopPosition() {
        return potentiometer.getVoltage() >= PotentiometerMaximum;
    }

    /**
     * Stops the arm from running
     */
    private void stop() {
        setState(ArmState.IDLE);
        motor1.setPower(0);
        motor2.setPower(0);
    }

    /**
     * Releases the pin of that holds the arm up when latching
     */
    public void releaseArmPin() {
        latchVoltage = potentiometer.getVoltage();
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
     *
     */
    public void slowFall() {
        //Ramp ramp = new LogarithmicRamp(new Point(0.0001, 0), new Point(latchVoltage, MaxPower));
        if (!hasHitFloor()) {
            motor1.setPower(0.2);
            motor2.setPower(-0.2);
        } else {
            setState(ArmState.RELEASE_PIN);
        }
    }

    private boolean hasHitFloor() {
        return potentiometer.getVoltage() >= 1.8 && potentiometer.getVoltage() <= 2.0;
    }
}
