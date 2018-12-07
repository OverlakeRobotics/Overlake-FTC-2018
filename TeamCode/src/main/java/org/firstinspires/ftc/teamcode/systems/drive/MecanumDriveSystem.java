package org.firstinspires.ftc.teamcode.systems.drive;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.components.configs.ConfigParser;
import org.firstinspires.ftc.teamcode.components.scale.ExponentialRamp;
import org.firstinspires.ftc.teamcode.components.scale.IScale;
import org.firstinspires.ftc.teamcode.components.scale.LinearScale;
import org.firstinspires.ftc.teamcode.components.scale.Point;
import org.firstinspires.ftc.teamcode.components.scale.Ramp;
import org.firstinspires.ftc.teamcode.systems.color.ColorSystem;
import org.firstinspires.ftc.teamcode.systems.imu.IMUSystem;

import java.util.Arrays;
import java.util.Collections;
import java.util.List;

public class MecanumDriveSystem extends DriveSystem4Wheel
{
    private ConfigParser config;

    public int TICKS_IN_INCH;
    public int TICKS_IN_INCH_STRAFE;
    private final IScale JOYSTICK_SCALE = new LinearScale(0.62, 0);
    private double TURN_RAMP_POWER_CUTOFF;
    public double RAMP_POWER_CUTOFF;
    public int RAMP_DISTANCE_TICKS;

    public IMUSystem imuSystem;

    private double initialHeading;
    private double initPitch;
    private double initRoll;

    private boolean slowDrive;

    /**
     * Constructs a new MecanumDriveSystem object.
     * @param opMode
     */
    public MecanumDriveSystem(OpMode opMode) {
        super(opMode);

        //this.config = new ConfigParser("Testy.omc");
        TICKS_IN_INCH = 69;
        TICKS_IN_INCH_STRAFE = 69;
        TURN_RAMP_POWER_CUTOFF = 0.1;
        RAMP_POWER_CUTOFF = 0.3;
        RAMP_DISTANCE_TICKS = inchesToTicks(20);

        imuSystem = new IMUSystem(opMode);
        initialHeading = imuSystem.getHeading();
        initPitch = imuSystem.getPitch();
        initRoll = imuSystem.getRoll();

        telemetry.log("MecanumDriveSystem","power: {0}", 0);
        telemetry.log("MecanumDriveSystem","distance: {0}", 0);
    }

    /**
     * Clips joystick values and drives the motors.
     * @param rightX Right X joystick value
     * @param rightY Right Y joystick value
     * @param leftX Left X joystick value
     * @param leftY Left Y joystick value in case you couldn't tell from the others
     * @param slowDrive Set to true for 30 % motor power.
     */
    public void mecanumDrive(float rightX, float rightY, float leftX, float leftY, boolean slowDrive) {
        this.slowDrive = slowDrive;
        setDirection(DriveDirection.FORWARD);
        rightX = Range.clip(rightX, -1, 1);
        leftX = Range.clip(leftX, -1, 1);
        leftY = Range.clip(leftY, -1, 1);

        rightX = scaleJoystickValue(rightX);
        leftX = scaleJoystickValue(leftX);
        leftY = scaleJoystickValue(leftY);

        // write the values to the motors 1
        /*double frontRightPower = -leftY + leftX - rightX; // left stick up moves forward
        double backRightPower = -leftY - leftX - rightX;
        double frontLeftPower = -leftY - leftX + rightX;
        double backLeftPower = -leftY + leftX + rightX;*/

        double frontRightPower = leftY - leftX - rightX; // left stick up strafes toward side where
        double backRightPower = -leftY - leftX - rightX; // arm is mounted
        double frontLeftPower = -leftY - leftX + rightX;  // lick left knee cap daddy pimple
        double backLeftPower = leftY - leftX + rightX;

        this.motorFrontRight.setPower(Range.clip(frontRightPower, -1, 1));
        this.motorBackRight.setPower(Range.clip(backRightPower, -1, 1));
        this.motorFrontLeft.setPower(Range.clip(frontLeftPower, -1, 1));
        this.motorBackLeft.setPower(Range.clip(backLeftPower, -1, 1));
    }

    /**
     * Scales the joystick value while keeping in mind slow mode.
     * @param joystickValue
     * @return a value from 0 - 1 based on the given value
     */
    private float scaleJoystickValue(float joystickValue) {
        float slowDriveCoefficient = .3f;
        if (!slowDrive) slowDriveCoefficient = 1;
        return joystickValue > 0
                ? (float)  JOYSTICK_SCALE.scaleX(joystickValue * joystickValue) * slowDriveCoefficient
                : (float) -JOYSTICK_SCALE.scaleX(joystickValue * joystickValue) * slowDriveCoefficient;
    }

    /**
     * Drives in God Mode (keeping angles consistent with the heading so it doesn't turn unexpectedly)
     * @param rightX Right X value of the joystick
     * @param rightY Right Y value of the joystick
     * @param leftX Left X value of the joystick
     * @param leftY Left Y value of the joystick in case that wasn't clear
     */
    public void driveGodMode(float rightX, float rightY, float leftX, float leftY) {
        driveGodMode(rightX, rightY, leftX, leftY, 1);
    }

    /**
     * Drives in God Mode (keeping angles consistent with the heading so it doesn't turn unexpectedly)
     * @param rightX Right X value of the joystick
     * @param rightY Right Y value of the joystick
     * @param leftX Left X value of the joystick
     * @param leftY Left Y value of the joys
     * @param coeff Used to set speed
     */
    public void driveGodMode(float rightX, float rightY, float leftX, float leftY, float coeff) {
        double currentHeading = Math.toRadians(imuSystem.getHeading());
        double headingDiff = initialHeading - currentHeading;

        rightX = scaleJoystickValue(rightX);
        leftX = scaleJoystickValue(leftX);
        leftY = scaleJoystickValue(leftY);

        double speed = Math.sqrt(leftX * leftX + leftY * leftY);
        double angle = Math.atan2(leftX, leftY) + (Math.PI / 2) + headingDiff;
        double changeOfDirectionSpeed = rightX;
        double x = coeff * speed * Math.cos(angle);
        double y = coeff * speed * Math.sin(angle);

        double frontLeft = Range.clip(y - changeOfDirectionSpeed + x, -1, 1);
        double frontRight = Range.clip(y + changeOfDirectionSpeed - x, -1, 1);
        double backLeft = Range.clip(y - changeOfDirectionSpeed - x, -1, 1);
        double backRight = Range.clip(y + changeOfDirectionSpeed + x, -1, 1);

        motorFrontLeft.setPower(frontLeft);
        motorFrontRight.setPower(frontRight);
        motorBackLeft.setPower(backLeft);
        motorBackRight.setPower(backRight);
    }

    public void mecanumDriveXY(double x, double y) {
        this.motorFrontRight.setPower(Range.clip(y + x, -1, 1));
        this.motorBackRight.setPower(Range.clip(y - x, -1, 1));
        this.motorFrontLeft.setPower(Range.clip(y - x, -1, 1));
        this.motorBackLeft.setPower(Range.clip(y + x, -1, 1));
    }

    /**
     * Drives using a polar coordinate system
     * @param radians the radians value
     * @param power The power of the motors
     */
    public void mecanumDrivePolar(double radians, double power) {
        double x = Math.cos(radians) * power;
        double y = Math.sin(radians) * power;
        mecanumDriveXY(x, y);
    }

    /**
     * Drives the given amount of inches forward.
     * @param inches Amount of inches to drive
     * @param power Power of the motors
     */
    public void driveToPositionInches(int inches, double power) {
        driveToPositionInches(inches, power, true);
    }

    public void driveToPositionInches(int inches, double power, boolean shouldRamp) {
        DriveDirection driveDirection = (power <= 0 || inches <= 0) ? DriveDirection.BACKWARD:
                DriveDirection.FORWARD;
        setDirection(driveDirection);
        int ticks = (int) inchesToTicks(inches);
        driveToPositionTicks(ticks, power, shouldRamp);
    }

    /**
     * Strafes left for the given amount of inches
     * @param inches Amount of inches to strafe
     * @param power Power of the motors
     */
    public void strafeLeftToPositionInches(int inches, double power) {
        setDirection(MecanumDriveDirection.STRAFE_LEFT);
        int ticks = (int) inchesToTicksStrafe(inches);
        driveToPositionTicks(ticks, power, true);
    }

    /**
     * Strafes right for the given amount of inches
     * @param inches Amount of inches to strafe
     * @param power Power of the motors
     */
    public void strafeRightToPositionInches(int inches, double power) {
        setDirection(MecanumDriveDirection.STRAFE_RIGHT);
        int ticks = (int) inchesToTicksStrafe(inches);
        driveToPositionTicks(ticks, power, true);
    }

    private void driveToPositionTicks(int ticks, double power, boolean shouldRamp) {
        setPower(0);

        motorFrontRight.setTargetPosition(motorFrontRight.getCurrentPosition() + ticks);
        motorFrontLeft.setTargetPosition(motorFrontLeft.getCurrentPosition() + ticks);
        motorBackRight.setTargetPosition(motorBackRight.getCurrentPosition() + ticks);
        motorBackLeft.setTargetPosition(motorBackLeft.getCurrentPosition() + ticks);

        setRunMode(DcMotor.RunMode.RUN_TO_POSITION);

        Ramp ramp = new ExponentialRamp(new Point(0, RAMP_POWER_CUTOFF),
                new Point(RAMP_DISTANCE_TICKS, Math.abs(power)));

        double adjustedPower = Range.clip(power, -1.0, 1.0);
        setPower(adjustedPower);

        while (anyMotorsBusy()) {
            int distance = getMinDistanceFromTarget();

            if (distance < 50) {
                break;
            }

            double direction = 1.0;
            if (distance < 0) {
                distance = -distance;
                direction = -1.0;
            }

            double scaledPower = shouldRamp ? ramp.scaleX(distance) : power;

            setPower(direction * scaledPower);
            telemetry.log("MecanumDriveSystem", "distance left (ticks): " + getMinDistanceFromTarget());
            telemetry.log("MecanumDriveSystem","scaled power: " + scaledPower);
            telemetry.write();
        }
        setPower(0);
    }

    /**
     * Gets the minimum distance from the target
     * @return
     */
    public int  getMinDistanceFromTarget() {
        int d = this.motorFrontLeft.getTargetPosition() - this.motorFrontLeft.getCurrentPosition();
        d = Math.min(d, this.motorFrontRight.getTargetPosition() - this.motorFrontRight.getCurrentPosition());
        d = Math.min(d, this.motorBackLeft.getTargetPosition() - this.motorBackLeft.getCurrentPosition());
        d = Math.min(d, this.motorBackRight.getTargetPosition() - this.motorBackRight.getCurrentPosition());
        return d;
    }

    /**
     * Converts inches to ticks
     * @param inches Inches to convert to ticks
     * @return
     */
    public int inchesToTicks(int inches) {
        return inches * TICKS_IN_INCH;
    }

    /**
     * Converts inches to ticks but for strafing
     * @param inches Inches to convert to ticks
     * @return
     */
    public double inchesToTicksStrafe(int inches) {
        return inches * TICKS_IN_INCH_STRAFE;
    }

    /**
     * Turns relative the heading upon construction
     * @param degrees The degrees to turn the robot by
     * @param maxPower The maximum power of the motors
     */
    public void turnAbsolute(double degrees, double maxPower) {
        turn(degrees, maxPower, initialHeading);
    }

    /**
     * Turns the robot by a given amount of degrees using the current heading
     * @param degrees The degrees to turn the robot by
     * @param maxPower The maximum power of the motors
     */
    public void turn(double degrees, double maxPower) {
        turn(degrees, maxPower, imuSystem.getHeading());
    }

    /**
     * Turns the robot by a given amount of degrees
     * @param degrees The degrees to turn the robot by
     * @param maxPower The maximum power of the motors
     * @param initialHeading The initial starting point
     */
    private void turn(double degrees, double maxPower, double initialHeading) {
        setDirection(DriveDirection.FORWARD);

        double heading = -initialHeading;
        double targetHeading = 0;

        if ((degrees % 360) > 180) {
            targetHeading = heading + ((degrees % 360) - 360);
        } else {
            targetHeading = heading + (degrees % 360);
        }

        setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Between 90 (changed from 130) and 2 degrees away from the target
        // we want to slow down from maxPower to 0.1
        ExponentialRamp ramp = new ExponentialRamp(new Point(2.0, TURN_RAMP_POWER_CUTOFF), new Point(90, maxPower));

        while (Math.abs(computeDegreesDiff(targetHeading, heading)) > 1) {
            double power = getTurnPower(ramp, targetHeading, heading);
            telemetry.log("MecanumDriveSystem","heading: " + heading);
            telemetry.log("MecanumDriveSystem","target heading: " + targetHeading);
            telemetry.log("MecanumDriveSystem","power: " + power);
            telemetry.log("MecanumDriveSystem","distance left: " + Math.abs(targetHeading - heading));
            telemetry.write();


            tankDrive(power, -power);
            heading = -imuSystem.getHeading();
        }
        this.setPower(0);
    }

    /**
     * Gets the turn power needed
     * @param ramp the ramp
     * @param targetHeading the target heading
     * @param heading the heading
     * @return
     */
    private double getTurnPower(Ramp ramp, double targetHeading, double heading) {
        double diff = computeDegreesDiff(targetHeading, heading);

        if (diff < 0) {
            return -ramp.scaleX(Math.abs(diff));
        } else {
            return ramp.scaleX(Math.abs(diff));
        }
    }

    private double computeDegreesDiff(double targetHeading, double heading) {
        return targetHeading - heading;
    }

    public void setDirection(MecanumDriveDirection direction) {
        switch (direction){
            case STRAFE_LEFT:
                motorFrontRight.setDirection(DcMotorSimple.Direction.FORWARD);
                motorFrontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
                motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);
                motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
                break;
            case STRAFE_RIGHT:
                motorFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
                motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
                motorBackRight.setDirection(DcMotorSimple.Direction.FORWARD);
                motorBackLeft.setDirection(DcMotorSimple.Direction.FORWARD);
                break;
        }
    }

    /**
     * Use STRAFE_RIGHT and STRAFE_LEFT for the setDirection() method
     */
    public enum MecanumDriveDirection {
        STRAFE_RIGHT, STRAFE_LEFT;
    }
}
