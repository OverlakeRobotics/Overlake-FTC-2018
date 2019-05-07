package org.firstinspires.ftc.teamcode.systems.base;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * This class is mostly used by MeccanumDriveSystem to avoid redundancy  - most methods simply take
 * a value and assign it to all four motors in some way or anohter.
 */
public class DriveSystem4Wheel extends System
{

    public DcMotor motorFrontLeft;
    public DcMotor motorFrontRight;
    public DcMotor motorBackLeft;
    public DcMotor motorBackRight;

    /**
     * Constructs a new DriveSystem4Wheel object.
     * @param opMode The OpMode to be used for constructing.
     */
    public DriveSystem4Wheel(OpMode opMode) {
        super(opMode, "DriveSystem4Wheel");

        this.motorFrontLeft = hardwareMap.dcMotor.get("motorFL");
        this.motorFrontRight = hardwareMap.dcMotor.get("motorFR");
        this.motorBackRight = hardwareMap.dcMotor.get("motorBR");
        this.motorBackLeft = hardwareMap.dcMotor.get("motorBL");

        setDirection(DriveDirection.FORWARD);

        this.motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        this.motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // Set all drive motors to zero power
        setPower(0);
    }

    /**
     * Sets the power of each motor.
     * Precondition: -1 < power < 1.
     * @param power The level of power for each motor - should be between -1 and 1.
     */
    public void setPower(double power) throws IllegalArgumentException {
        if (power < -1 || power > 1) {
            throw new IllegalArgumentException();
        }
        this.motorFrontLeft.setPower(power);
        this.motorFrontRight.setPower(power);
        this.motorBackLeft.setPower(power);
        this.motorBackRight.setPower(power);
    }

    /**
     * Returns true if any motors are busy.
     * @return
     */
    public boolean anyMotorsBusy()
    {
        return motorFrontLeft.isBusy() ||
                motorFrontRight.isBusy() ||
                motorBackLeft.isBusy() ||
                motorBackRight.isBusy();
    }

    /**
     * Sets the target postition in ticks.
     * @param ticks The number of ticks.
     */
    public void setTargetPosition(int ticks)
    {
        motorBackLeft.setTargetPosition(motorBackLeft.getCurrentPosition() + ticks);
        motorBackRight.setTargetPosition(motorBackRight.getCurrentPosition() + ticks);
        motorFrontLeft.setTargetPosition(motorFrontLeft.getCurrentPosition() + ticks);
        motorFrontRight.setTargetPosition(motorFrontRight.getCurrentPosition() + ticks);
    }

    /**
     * Sets the mode of all motors to the given RunMode.
     * @param runMode
     */
    public void setRunMode(DcMotor.RunMode runMode)
    {
        // lick left kneecap daddy pimple
        motorFrontLeft.setMode(runMode);
        motorFrontRight.setMode(runMode);
        motorBackLeft.setMode(runMode);
        motorBackRight.setMode(runMode);
    }

    /**
     * Sets direction of robot using DriveDirection enum.
     * @param direction The direction of robot. Use DriveDirection.FORWARD, DriveDirection.REVERSE,
     *                  or DriveDirection.ALL_FORWARD.
     */
    public void setDirection(DriveDirection direction) {
        DcMotorSimple.Direction forward = DcMotorSimple.Direction.FORWARD; // Avoid Redundancy
        DcMotorSimple.Direction reverse = DcMotorSimple.Direction.REVERSE;
        switch (direction){
            case FORWARD:
                motorFrontRight.setDirection(reverse);
                motorFrontLeft.setDirection(reverse);
                motorBackRight.setDirection(forward);
                motorBackLeft.setDirection(forward);
                break;
            case REVERSE:
                motorFrontRight.setDirection(forward);
                motorFrontLeft.setDirection(forward);
                motorBackRight.setDirection(reverse);
                motorBackLeft.setDirection(reverse);
                break;
            case ALL_FORWARD:
                motorBackRight.setDirection(forward);
                motorFrontRight.setDirection(forward);
                motorBackLeft.setDirection(forward);
                motorFrontLeft.setDirection(forward);
                break;
        }
    }

    public enum DriveDirection {
        FORWARD, REVERSE, ALL_FORWARD;
    }
}
