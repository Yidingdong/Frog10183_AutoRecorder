/*
 * MIT License — Copyright (c) 2026 FTC Team 10183 F.R.O.G. See LICENSE.
 */

package org.firstinspires.ftc.teamcode.universal;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * ─────────────────────────────────────────────────────────────────────────────
 *  EXAMPLE ONLY — a minimal TeleOp that shows how to wire in UniversalAutoRecorder.
 *
 *  The motor names ("leftFront", "rightFront", "leftBack", "rightBack") and the
 *  mecanum drive math below are ILLUSTRATIVE. To run this as-is, create a robot
 *  configuration with those exact names. Otherwise, change the names / drive code
 *  to match your robot, OR just copy the recorder calls into your own TeleOp —
 *  you do NOT need this file to use the library.
 *
 *  Motor DIRECTIONS are robot-specific: most mecanum builds need one side reversed.
 *  Verify with a quick drive test and flip the setDirection calls if a wheel spins
 *  the wrong way.
 * ─────────────────────────────────────────────────────────────────────────────
 */
@TeleOp(name = "Sample Recorder TeleOp", group = "Universal")
@Disabled // example only — remove this line to try it on the Driver Station
public class SampleRecorderTeleOp extends LinearOpMode {

    private UniversalAutoRecorder recorder;

    @Override
    public void runOpMode() {
        // --- Your robot hardware (rename to match your config) ---
        DcMotor leftFront  = hardwareMap.get(DcMotor.class, "leftFront");
        DcMotor rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        DcMotor leftBack   = hardwareMap.get(DcMotor.class, "leftBack");
        DcMotor rightBack  = hardwareMap.get(DcMotor.class, "rightBack");

        // Typical mecanum: reverse the right side. CHANGE IF A WHEEL SPINS WRONG.
        leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        leftBack.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);

        // --- Recorder: line 1 of 3 (construct AFTER hardware is set up) ---
        recorder = new UniversalAutoRecorder(hardwareMap, gamepad1, telemetry, this);
        recorder.waitForSlotSelection();   // blocking slot-picker on the Driver Station

        waitForStart();

        try {
            while (opModeIsActive()) {
                // --- Robot-centric mecanum drive on gamepad1 ---
                double y  = -gamepad1.left_stick_y;  // forward
                double x  =  gamepad1.left_stick_x;  // strafe
                double rx =  gamepad1.right_stick_x;  // turn

                double denom = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1.0);
                leftFront.setPower((y + x + rx) / denom);
                leftBack.setPower((y - x + rx) / denom);
                rightFront.setPower((y - x - rx) / denom);
                rightBack.setPower((y + x - rx) / denom);

                // --- Recorder: line 2 of 3 (safe to call every loop) ---
                recorder.capture();
                recorder.showTelemetry();
                telemetry.update();
            }
        } finally {
            // --- Recorder: line 3 of 3 — in finally so the recording is saved even if
            // the loop exits via an exception, not only a normal STOP ---
            recorder.close();
        }
    }
}
