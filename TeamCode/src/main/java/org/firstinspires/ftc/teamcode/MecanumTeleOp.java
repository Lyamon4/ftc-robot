package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Mecanum drive, field-centric: pushing the left stick forward always drives
 * away from the driver, whichever way the robot faces.
 *
 * Robot configuration names (set them on the Driver Station):
 *   motors: frontLeft, frontRight, backLeft, backRight
 *   imu:    imu (the one built into the Control Hub)
 *
 * Gamepad 1:
 *   left stick  - drive and strafe
 *   right stick - turn
 *   right bumper (held) - slow mode for precise moves
 *   options     - "forward is where the robot faces now" (reset heading and position)
 *
 * The robot's position on the field (from the wheel encoders and the IMU) is
 * shown in telemetry.
 */
@TeleOp(name = "Mecanum TeleOp", group = "Drive")
public class MecanumTeleOp extends LinearOpMode {

    /** Speed while slow mode is held. */
    private static final double SLOW = 0.35;

    @Override
    public void runOpMode() {
        // Read all motor encoders and sensors of each hub in one request per loop.
        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        DcMotor frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        DcMotor frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        DcMotor backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        DcMotor backRight = hardwareMap.get(DcMotor.class, "backRight");

        // Left side motors are mounted mirrored. If the robot spins instead of
        // driving forward, flip these to the right side instead.
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        for (DcMotor m : new DcMotor[] {frontLeft, frontRight, backLeft, backRight}) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        // How the Control Hub is mounted on the robot: change to match yours.
        IMU imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)));
        imu.resetYaw();
        MecanumOdometry odometry = new MecanumOdometry(frontLeft, frontRight, backLeft, backRight, imu);

        telemetry.addLine("Ready. Left stick: drive, right stick: turn, RB: slow, options: reset heading.");
        telemetry.update();
        waitForStart();

        ElapsedTime loopTimer = new ElapsedTime();
        while (opModeIsActive()) {
            if (gamepad1.options) {
                imu.resetYaw();
                odometry.setPose(0, 0, 0);
            }
            odometry.update();

            // Sticks: up is negative on the gamepad, so flip y.
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double turn = gamepad1.right_stick_x;

            // Turn the stick direction from the field's frame into the robot's.
            double heading = odometry.heading();
            double rotX = x * Math.cos(-heading) - y * Math.sin(-heading);
            double rotY = x * Math.sin(-heading) + y * Math.cos(-heading);
            rotX *= 1.1; // strafing is a bit weaker on mecanum wheels

            // Keep the ratios when any wheel would go past full power.
            double scale = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(turn), 1.0);
            double speed = gamepad1.right_bumper ? SLOW : 1.0;

            frontLeft.setPower(speed * (rotY + rotX + turn) / scale);
            backLeft.setPower(speed * (rotY - rotX + turn) / scale);
            frontRight.setPower(speed * (rotY - rotX - turn) / scale);
            backRight.setPower(speed * (rotY + rotX - turn) / scale);

            telemetry.addData("position", "X %.1f in, Y %.1f in", odometry.x(), odometry.y());
            telemetry.addData("heading", "%.1f°", Math.toDegrees(heading));
            telemetry.addData("loop", "%.1f ms", loopTimer.milliseconds());
            telemetry.update();
            loopTimer.reset();
        }
    }
}
