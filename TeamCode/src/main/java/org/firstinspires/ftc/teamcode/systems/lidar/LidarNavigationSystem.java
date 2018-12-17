package org.firstinspires.ftc.teamcode.systems.lidar;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.components.scale.ExponentialRamp;
import org.firstinspires.ftc.teamcode.components.scale.Point;
import org.firstinspires.ftc.teamcode.components.scale.Ramp;
import org.firstinspires.ftc.teamcode.opmodes.IBaseOpMode;
import org.firstinspires.ftc.teamcode.systems.drive.MecanumDriveSystem;
import org.firstinspires.ftc.teamcode.systems.drive.DriveSystem4Wheel;
import org.firstinspires.ftc.teamcode.systems.System;
import org.firstinspires.ftc.teamcode.systems.color.ColorSystem;
import org.firstinspires.ftc.teamcode.systems.imu.IMUSystem;

public class LidarNavigationSystem extends System {
    DistanceSensor lidar;
    DistanceSensor lidar2;
    Rev2mDistanceSensor sensorTimeOfFlight;
    HardwareMap hwmap;
    MecanumDriveSystem driveSystem;
    IMUSystem imu;
    ColorSystem colorSystem;
    OpMode opMode;

    double initPitch;
    double initRoll;

    public LidarNavigationSystem(IBaseOpMode opMode, MecanumDriveSystem driveSystem, ColorSystem colorSystem) {
        super(opMode, "DriveSystem4Wheel");
        this.hwmap = opMode.getHardwareMap();
        this.driveSystem = driveSystem;
        imu = new IMUSystem(opMode);
        this.colorSystem = colorSystem;
        initSystem();

        initPitch = imu.getPitch();
        initRoll = imu.getRoll();
    }

    public void initSystem() {
        lidar = hwmap.get(DistanceSensor.class, "sensor_range");
        lidar2 = hwmap.get(DistanceSensor.class, "sensor_range2");
        Rev2mDistanceSensor sensorTimeOfFlight = (Rev2mDistanceSensor)lidar;
    }

    public double getDistance1() {
        return lidar.getDistance(DistanceUnit.INCH);
    }

    public double getDistance2() {
        return lidar2.getDistance(DistanceUnit.INCH);
    }

    public void telemetry() {
        log.info("range1", String.format("%.01f in", lidar.getDistance(DistanceUnit.INCH)));
        log.info("range2", String.format("%.01f in", lidar2.getDistance(DistanceUnit.INCH)));
        log.info("frontRightDistance: ", "" + getFrontRightDistance());
    }

    public double getFrontRightDistance() {
        double d1 = getDistance1();
        double d2 = getDistance2();
        double df = d1 - d2;
        double ds = 10.0; // inches
        double d = (d1 * (Math.sin(Math.atan((ds/df)))));
        return d;
    }

    public void driveAlongWallInches(int inches, double closeBuffer, double farBuffer, double power,
                                     boolean shouldRamp) {
        int ticks = driveSystem.inchesToTicks(inches);
        driveSystem.setDirection(DriveSystem4Wheel.DriveDirection.FORWARD);
        driveSystem.setPower(0);

        driveSystem.motorFrontRight.setTargetPosition(driveSystem.motorFrontRight.getCurrentPosition() + ticks);
        driveSystem.motorFrontLeft.setTargetPosition(driveSystem.motorFrontLeft.getCurrentPosition() + ticks);
        driveSystem.motorBackRight.setTargetPosition(driveSystem.motorBackRight.getCurrentPosition() + ticks);
        driveSystem.motorBackLeft.setTargetPosition(driveSystem.motorBackLeft.getCurrentPosition() + ticks);

        driveSystem.setRunMode(DcMotor.RunMode.RUN_TO_POSITION);

        Ramp ramp = new ExponentialRamp(new Point(0, driveSystem.RAMP_POWER_CUTOFF),
                new Point(driveSystem.RAMP_DISTANCE_TICKS, power));

        double adjustedPower = Range.clip(power, -1.0, 1.0);

        driveSystem.setPower(adjustedPower);

        while (driveSystem.anyMotorsBusy()) {
            int distance = driveSystem.getMinDistanceFromTarget();

            if (distance < 50) {
                break;
            }

            correctToFollowWall(closeBuffer, farBuffer, power, StopCondition.DISTANCE);

            double direction = 1.0;
            if (distance < 0) {
                distance = -distance;
                direction = -1.0;
            }

            double scaledPower = shouldRamp ? ramp.scaleX(distance) : power;
            log.info("LidarNavigationSystem",
                    "ticks left (ticks): " + driveSystem.getMinDistanceFromTarget());
            log.info("LidarNavigationSystem","scaled power: " + scaledPower);
            log.info("LidarNavigationSystem", "distance1: " + getDistance1());
            log.info("LidarNavigationSystem", "distance2: " + getDistance2());
            driveSystem.setPower(direction * scaledPower);
        }
        driveSystem.setPower(0);
    }

    private void correctToFollowWall(double closeBuffer, double farBuffer, double correctionPower,
                                    StopCondition stopCondition) {
        boolean isOutOfBounds = isOutOfBounds(closeBuffer, farBuffer);
        if (isOutOfBounds) {
            driveSystem.setPower(0);
            log.info("driveTest", "distance buffer triggered");
        }
        while (isOutOfBounds(closeBuffer, farBuffer)) {
            log.info("driveTest", "isOutOfBounds");

            double[] correctionPowers = getCorrectionTurnPower(closeBuffer, farBuffer, correctionPower);
            double leftPower = correctionPowers[0];
            double rightPower = correctionPowers[1];

            driveSystem.tankDrive(leftPower, rightPower);
            log.info("LidarNavigationSystem", "leftPower: " + leftPower);
            log.info("LidarNavigationSystem", "rightPower: " +  rightPower);
        }
    }

    private boolean isOutOfBounds(double closeBuffer, double farBuffer) {
        return (getDistance1() >= farBuffer) ||
                (getDistance2() >= farBuffer) ||
                (getDistance1() <= closeBuffer) ||
                (getDistance2() <= closeBuffer);
    }

    public void driveAlongWallToCrater(double closeBuffer, double farBuffer, double power) {
        driveSystem.setDirection(DriveSystem4Wheel.DriveDirection.FORWARD);
        driveSystem.setPower(0);

        driveSystem.setRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        double adjustedPower = Range.clip(power, -1.0, 1.0);
        driveSystem.setPower(adjustedPower);

        while (!isOnCrater()) {
            correctToFollowWall(closeBuffer, farBuffer, power, StopCondition.CRATOR);

            log.info("LidarNavigationSystem","scaled power: " + adjustedPower);
            driveSystem.setPower(adjustedPower);
        }
        driveSystem.setPower(0);
        log.info("LidarNavigationSystem","reached crator");
    }

    public void driveAlongWallToDepot(double closeBuffer, double farBuffer, double power) {
        driveSystem.setDirection(DriveSystem4Wheel.DriveDirection.FORWARD);
        driveSystem.setPower(0);

        driveSystem.setRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        double adjustedPower = Range.clip(power, -1.0, 1.0);
        driveSystem.setPower(adjustedPower);

        while (!isInDepot()) {
            correctToFollowWall(closeBuffer, farBuffer, power, StopCondition.CRATOR);

            log.info("LidarNavigationSystem","scaled power: " + adjustedPower);
            driveSystem.setPower(adjustedPower);
        }
        driveSystem.setPower(0);
        log.info("LidarNavigationSystem","reached depot");
    }

    public void getCloseToWall(double targetDistanceFromWall, double power) {
        driveSystem.setPower(power);

        while (isGettingCloseToWall(targetDistanceFromWall)) {
            driveSystem.setPower(power);
            log.info("LidarNavigationSystem", "distance 1: " + getDistance1());
            log.info("LidarNavigationSystem", "distance 2: " + getDistance2());
        }
        driveSystem.setPower(0);
    }

    public boolean isGettingCloseToWall(double targetDistanceFromWall) {
        return (getDistance2() > targetDistanceFromWall) && (getDistance1() > targetDistanceFromWall);
    }

    public void strafeTowardWallPolar(double targetDistanceFromWall, double wallHeading, double driveHeading,
                                      double power) {
        driveSystem.turnAbsolute(wallHeading, power);
        driveSystem.setDirection(DriveSystem4Wheel.DriveDirection.FORWARD);
        driveSystem.setRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        driveSystem.mecanumDrivePolar(driveHeading, power);

        while (isGettingCloseToWall(targetDistanceFromWall)) {
            driveSystem.mecanumDrivePolar(driveHeading, power);
        }
        driveSystem.setPower(0);
    }

    public void strafeTowardWall(double targetDistanceFromWall, double wallHeading, double power) {
        driveSystem.turnAbsolute(wallHeading, power);
        driveSystem.setDirection(MecanumDriveSystem.MecanumDriveDirection.STRAFE_LEFT);
        driveSystem.setPower(power);

        while (isGettingCloseToWall(targetDistanceFromWall)) {
            driveSystem.setPower(power);
        }
        driveSystem.setPower(0);
    }

    private double[] getCorrectionTurnPower(double closeBuffer, double farBuffer, double power) {
        double highPoweredSideturnPower = (power / 2);
        double lowPoweredSideTurnPower = (power / 5);
        double rightPower = highPoweredSideturnPower;
        double leftPower = highPoweredSideturnPower;
        if ((((getDistance1() >= farBuffer) || getDistance2() <= closeBuffer) && !(getDistance2() >= farBuffer)) ||
                ((getDistance1() <= farBuffer) && (getDistance2() <= farBuffer))) {
            if (power > 0) {
//                telemetry.log("driveTest", "turning RIGHT");
                rightPower = lowPoweredSideTurnPower;
                leftPower = highPoweredSideturnPower;
            } else {
//                telemetry.log("driveTest", "turning LEFT");
                rightPower = highPoweredSideturnPower;
                leftPower = lowPoweredSideTurnPower;
            }
        } else if ((((getDistance2() >= farBuffer) || (getDistance1() <= closeBuffer))) ||
                ((getDistance1() >= farBuffer) && (getDistance2() >= farBuffer))) {
            if (power > 0) {
//                telemetry.log("driveTest", "turning LEFT");
                rightPower = highPoweredSideturnPower;
                leftPower = lowPoweredSideTurnPower;
            } else {
//                telemetry.log("driveTest", "turning RIGHT");
                rightPower = lowPoweredSideTurnPower;
                leftPower = highPoweredSideturnPower;
            }
        }
        double[] powers = new double[] {leftPower, rightPower};
//        telemetry.write();
        return powers;
    }

    public boolean isOnCrater() {
        double criticalAngle = 1.5;
        return ((Math.abs(imu.getPitch() - initPitch) < criticalAngle) ||
                (Math.abs(imu.getRoll() - initRoll) < criticalAngle));
    }

    public boolean isInDepot() {
        int redTriggerValue = 12;
        int blueTriggerValue = 8;
        return ((colorSystem.getRed() < redTriggerValue) ||
                (colorSystem.getBlue() < blueTriggerValue));
    }

    public void alignWithWall(double maxPower) {
        driveSystem.setDirection(DriveSystem4Wheel.DriveDirection.FORWARD);
        driveSystem.setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Between 90 (changed from 130) and 2 degrees away from the target
        // we want to slow down from maxPower to 0.1
        ExponentialRamp ramp = new ExponentialRamp(new Point(2.0, driveSystem.TURN_RAMP_POWER_CUTOFF), new Point(90, maxPower));

        while (Math.abs(getDistance1() - getDistance2()) > 0.05) {
            double power = driveSystem.getTurnPower(ramp, getDistance2(), getDistance1());
//            telemetry.log("MecanumDriveSystem","heading: " + (getDistance2() - getDistance1()));
//            telemetry.log("MecanumDriveSystem","power: " + power);
//            telemetry.write();
            driveSystem.tankDrive(power, -power);
        }
        driveSystem.setPower(0);
    }
}
