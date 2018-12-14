package org.firstinspires.ftc.teamcode.systems.slide;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.teamcode.components.scale.LogarithmicRamp;
import org.firstinspires.ftc.teamcode.components.scale.Point;
import org.firstinspires.ftc.teamcode.components.scale.Ramp;
import org.firstinspires.ftc.teamcode.opmodes.IBaseOpMode;
import org.firstinspires.ftc.teamcode.systems.System;

/**
 * Runs a linear slide system on the robot
 */
public class SlideSystem extends System
{
    private final double MaxWinchPower = 1;
    public final int EncoderTop = 8300;

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
    public SlideSystem(IBaseOpMode opMode)
    {
        super(opMode, "SlideSystem");
        winch = hardwareMap.dcMotor.get("winch");
        limitTop = hardwareMap.get(DigitalChannel.class, "limitTop");
        limitMiddle = hardwareMap.get(DigitalChannel.class, "limitMiddle");

        setState(SlideState.IDLE);
        limitTop.setMode(DigitalChannel.Mode.INPUT);
        limitMiddle.setMode(DigitalChannel.Mode.INPUT);

        winchOrigin = winch.getCurrentPosition();
        rampTop = new LogarithmicRamp(
                new Point(0.1,0),
                new Point((winchOrigin + EncoderTop), MaxWinchPower)
        );
        rampBottom = new LogarithmicRamp(
                new Point((winchOrigin + EncoderTop), MaxWinchPower),
                new Point(0.1, 0)
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
            //winch.setPower(rampTop.scaleX(winch.getCurrentPosition()));
            winch.setPower(MaxWinchPower);
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
    public boolean isAtTop() {
        return winch.getCurrentPosition() >= winchOrigin + EncoderTop;
    }

    /**
     * Slides down to the bottom
     */
    public void slideDown()
    {
        winch.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        if (!isAtBottom())
        {
            //winch.setPower(-rampBottom.scaleX(winch.getCurrentPosition()));
            winch.setPower(-MaxWinchPower);
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
        return winch.getCurrentPosition() < winchOrigin;
    }

    /**
     * Stops the slide
     */
    public void stop() {
        setState(SlideState.IDLE);
        winch.setPower(0);
    }

    /**
     * Sets the origin of the slide system
     */
    public void setOrigin(int origin) {
        this.winchOrigin = origin;
    }

    /**
     * Gets the position of the slide
     */
    public int getPosition() {
        return winch.getCurrentPosition();
    }
}
