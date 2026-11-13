package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * Where the robot is on the field, from the four drive motor encoders and the
 * IMU heading. Good enough to watch the robot on a field view and for simple
 * moves; for accurate autonomous use dedicated odometry (dead wheels or a
 * goBILDA Pinpoint), because mecanum wheels slip.
 *
 * Field frame: X forward and Y left as the robot faced when the pose was last
 * reset, in inches; heading counter-clockwise, in radians.
 *
 * The constants below are @Config: Swervely can tune them live.
 */
@Config
public class MecanumOdometry {

    /** goBILDA 104 mm mecanum wheel. */
    public static double WHEEL_DIAMETER_IN = 104.0 / 25.4;
    /** goBILDA Yellow Jacket 312 rpm (19.2:1) motor. Change for other gearboxes. */
    public static double TICKS_PER_REV = 537.7;
    /** Strafing travels less than the wheels turn; measure and adjust. */
    public static double STRAFE_FACTOR = 0.9;

    private final DcMotor frontLeft, frontRight, backLeft, backRight;
    private final IMU imu;
    private int lastFL, lastFR, lastBL, lastBR;
    private double x, y, heading, headingOffset;

    public MecanumOdometry(DcMotor frontLeft, DcMotor frontRight, DcMotor backLeft, DcMotor backRight, IMU imu) {
        this.frontLeft = frontLeft;
        this.frontRight = frontRight;
        this.backLeft = backLeft;
        this.backRight = backRight;
        this.imu = imu;
        setPose(0, 0, 0);
    }

    /** Say where the robot is now (inches, radians). */
    public void setPose(double x, double y, double heading) {
        this.x = x;
        this.y = y;
        this.headingOffset = heading - imuHeading();
        this.heading = heading;
        lastFL = frontLeft.getCurrentPosition();
        lastFR = frontRight.getCurrentPosition();
        lastBL = backLeft.getCurrentPosition();
        lastBR = backRight.getCurrentPosition();
    }

    /** Call once per loop. */
    public void update() {
        int fl = frontLeft.getCurrentPosition();
        int fr = frontRight.getCurrentPosition();
        int bl = backLeft.getCurrentPosition();
        int br = backRight.getCurrentPosition();
        double inPerTick = Math.PI * WHEEL_DIAMETER_IN / TICKS_PER_REV;
        double dFL = (fl - lastFL) * inPerTick, dFR = (fr - lastFR) * inPerTick;
        double dBL = (bl - lastBL) * inPerTick, dBR = (br - lastBR) * inPerTick;
        lastFL = fl;
        lastFR = fr;
        lastBL = bl;
        lastBR = br;

        // Mecanum forward kinematics, in the robot's own frame.
        double forward = (dFL + dFR + dBL + dBR) / 4.0;
        double right = (dFL - dFR - dBL + dBR) / 4.0 * STRAFE_FACTOR;

        // Turn the step into the field frame using the heading halfway through it.
        double newHeading = imuHeading() + headingOffset;
        double mid = heading + angleDiff(newHeading, heading) / 2.0;
        double left = -right;
        x += forward * Math.cos(mid) - left * Math.sin(mid);
        y += forward * Math.sin(mid) + left * Math.cos(mid);
        heading = newHeading;
    }

    public double x() {
        return x;
    }

    public double y() {
        return y;
    }

    /** Radians, counter-clockwise. */
    public double heading() {
        return heading;
    }

    private double imuHeading() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }

    /** a - b, wrapped to (-pi, pi]. */
    private static double angleDiff(double a, double b) {
        double d = a - b;
        while (d > Math.PI) d -= 2 * Math.PI;
        while (d <= -Math.PI) d += 2 * Math.PI;
        return d;
    }
}
