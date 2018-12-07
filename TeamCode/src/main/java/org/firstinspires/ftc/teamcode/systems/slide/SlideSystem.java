package org.firstinspires.ftc.teamcode.systems.slide;

import android.transition.Slide;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.teamcode.components.scale.ExponentialRamp;
import org.firstinspires.ftc.teamcode.components.scale.LogarithmicRamp;
import org.firstinspires.ftc.teamcode.components.scale.Point;
import org.firstinspires.ftc.teamcode.components.scale.Ramp;
import org.firstinspires.ftc.teamcode.systems.base.System;

/**
 * Runs a linear slide system on the robot
 */
public class SlideSystem extends System
{
    private final double MaxWinchPower = 0.7;
    private final int EncoderTop = 8730;

    private DigitalChannel limitTop;
    private DigitalChannel limitMiddle;
    private DcMotor winch;
    private Ramp rampTop;
    private Ramp rampBottom;
    private SlideState state;

    private int winchOrigin;

    /**
     * Creates a new linear slide system in the current opmode
     * @param opMode current opmode being executed
     */
    public SlideSystem(OpMode opMode)
    {
        super(opMode, "SlideSystem");
        winch = hardwareMap.dcMotor.get("winch");
        limitTop = hardwareMap.get(DigitalChannel.class, "limitTop");
        limitMiddle = hardwareMap.get(DigitalChannel.class, "limitMiddle");

        setState(SlideState.IDLE);
        limitTop.setMode(DigitalChannel.Mode.INPUT);
        limitMiddle.setMode(DigitalChannel.Mode.INPUT);

        winchOrigin = winch.getCurrentPosition();
        rampBottom = new LogarithmicRamp(
                new Point(0.001,0.1),
                new Point((winchOrigin + EncoderTop), MaxWinchPower)
        );
        rampTop = new LogarithmicRamp(
                new Point((winchOrigin + EncoderTop), MaxWinchPower),
                new Point(0.0001, 0.1)
        );
    }

    /**
     * Sets the state of the slide
     * @param state new state of the slide
     */
    public void setState(SlideState state) {
        this.state = state;
    }

    /**
     * Runs the slide system
     */
    public void run() {
        telemetry.log("Encoder", winch.getCurrentPosition());
        telemetry.log("Top Power", rampTop.scaleX(winch.getCurrentPosition()));
        telemetry.log("Bottom Power", rampBottom.scaleX(winch.getCurrentPosition()));
        telemetry.log("Origin", winchOrigin);
        telemetry.write();
        switch (state) {
            case WINCHING_TO_TOP:
                slideUp();
                break;
            case WINCHING_TO_BOTTOM:
                slideDown();
                break;
            default:
                stop();
                break;
        }
    }

    /**
     * Slides up to the top
     */
    public void slideUp()
    {
        winch.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        if (!isAtTop())
        {
            winch.setPower(rampTop.scaleX(winch.getCurrentPosition()));
//            winch.setPower(MaxWinchPower);
        }
        else
        {
            winch.setPower(0.0);
        }
    }

    /**
     * Checks if the slide is at the top
     * @return Returns true if the slide is at the top
     */
    private boolean isAtTop() {
        return !limitTop.getState() && !limitMiddle.getState();
    }

    /**
     * Slides down to the bottom
     */
    public void slideDown()
    {
        winch.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        if (!isAtBottom())
        {
            winch.setPower(-rampBottom.scaleX(winch.getCurrentPosition()));
//            winch.setPower(-MaxWinchPower);
        }
        else
        {
            winch.setPower(0);
        }
    }

    /**
     * Checks if the slide is at the bottom
     * @return Returns true if the slide is at the bottom
     */
    private boolean isAtBottom() {
        return winch.getCurrentPosition() <= winchOrigin;
    }

    /**
     * Stops the slide
     */
    public void stop() {
        setState(SlideState.IDLE);
        winch.setPower(0);
    }
}
