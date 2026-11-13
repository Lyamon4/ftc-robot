package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

/**
 * Drive values that can be changed live from Swervely's tuning panel while
 * the OpMode runs (public static fields in a @Config class).
 */
@Config
public class DriveTuning {
    /** Speed while slow mode (right bumper) is held. */
    public static double SLOW_SPEED = 0.35;
    /** Extra push on strafing: mecanum wheels strafe weaker than they drive. */
    public static double STRAFE_GAIN = 1.1;
}
