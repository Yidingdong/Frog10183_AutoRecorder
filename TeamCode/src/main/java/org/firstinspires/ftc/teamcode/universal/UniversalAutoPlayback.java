/*
 * MIT License
 *
 * Copyright (c) 2026 FTC Team 10183 F.R.O.G.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND. See LICENSE.
 */

package org.firstinspires.ftc.teamcode.universal;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

/**
 * Universal Auto Playback — replays any recording made by {@link UniversalAutoRecorder}.
 *
 * <p>Zero-config: reads the self-describing header to discover which devices to control,
 * reconstructs each in the control mode it was recorded in, then replays with
 * <b>linear interpolation</b> for smooth motion. Works on ANY FTC robot — no hardcoded
 * motor names.</p>
 *
 * <h3>Faithful replay</h3>
 * <ul>
 *   <li><b>Velocity motors</b> (recorded in {@code RUN_USING_ENCODER}) are reconstructed in
 *       velocity mode with their original PIDF and driven with {@code setVelocity(ticks/sec)},
 *       so a flywheel holds its RPM regardless of battery voltage.</li>
 *   <li><b>Power motors</b> are driven with {@code setPower}, optionally <b>battery-voltage
 *       compensated</b> per frame so a drained pack on playback day doesn't undershoot.</li>
 *   <li><b>Servos</b> replay absolute positions; <b>CR servos</b> replay power.</li>
 * </ul>
 *
 * <h3>Controls (init-time, before Start):</h3>
 * <pre>
 *   D-Pad UP/DOWN: Select slot (1-10)
 *   X:             Toggle battery-voltage compensation (default ON)
 *   START:         Begin playback
 * </pre>
 */
@Autonomous(name = "Universal Auto Playback", group = "Universal")
@Disabled // ships disabled — remove this line to show it on the Driver Station
public class UniversalAutoPlayback extends LinearOpMode {

    /** File format version understood by this playback. Must match UniversalAutoRecorder. */
    static final int FORMAT_VERSION = 2;

    private static final String STORAGE_DIR = "/sdcard/FIRST/autorecorder/";
    private static final int MAX_SLOTS = 10;
    private static final long SAFETY_TIMEOUT_MS = 30_000;
    private static final long LOOP_SLEEP_MS = 5;

    private enum DeviceType { MOTOR, SERVO, CRSERVO }
    private enum ChannelKind { POWER, VELOCITY }

    // Slot selection
    private int selectedSlot = 1;
    private boolean dpadUpWas;
    private boolean dpadDownWas;
    private boolean xWas;

    // Battery-voltage compensation
    private boolean voltageColumnPresent = false; // does the file carry a vbat column?
    private boolean voltageCompEnabled = true;     // user toggle (X)
    private VoltageSensor playbackVoltageSensor;   // null if this robot has none

    // Load status shown in the init-time selection UI
    private String loadError;
    private String loadWarning;

    private static class DeviceDef {
        DeviceType type;
        ChannelKind kind = ChannelKind.POWER; // motors only; servos/crservos ignore it
        String name;
        DcMotorSimple.Direction motorDir = DcMotorSimple.Direction.FORWARD;
        DcMotor.ZeroPowerBehavior motorZbp = DcMotor.ZeroPowerBehavior.BRAKE;
        double[] pidf;            // velocity PIDF, null unless present
        boolean wasRunToPosition; // recorded in RUN_TO_POSITION (replayed as power)
    }

    // Reconstructed devices (same order as data columns)
    private final List<DeviceDef> deviceDefs = new ArrayList<>();
    private final List<DcMotorEx> motors = new ArrayList<>();
    private final List<Servo> servos = new ArrayList<>();
    private final List<CRServo> crServos = new ArrayList<>();

    // Playback data (each frame: timestamp + optional vbat + one double per device)
    private final List<long[]> frameTimestamps = new ArrayList<>();
    private final List<double[]> frameData = new ArrayList<>();
    private final List<Double> frameVbat = new ArrayList<>();

    @Override
    public void runOpMode() {
        cachePlaybackVoltageSensor();
        selectAndLoadSlot();           // INIT: pick a slot and parse it (no motion yet)
        if (!opModeIsActive()) return; // STOP pressed during selection
        initDevices();                 // reconstruct hardware once, at START
        executePlayback();
    }

    // ========================
    // SLOT SELECTION + LOAD (init-time)
    // ========================

    /**
     * Init-time slot picker. The highlighted recording is parsed live (during INIT) so that
     * pressing START begins motion immediately and the on-screen summary is accurate.
     * Exits on START (Driver Station PLAY) or STOP.
     */
    private void selectAndLoadSlot() {
        int loadedSlot = -1;
        while (!isStarted() && !isStopRequested()) {
            if (gamepad1.dpad_up && !dpadUpWas) {
                selectedSlot--;
                if (selectedSlot < 1) selectedSlot = MAX_SLOTS;
                dpadUpWas = true;
            } else if (!gamepad1.dpad_up) {
                dpadUpWas = false;
            }

            if (gamepad1.dpad_down && !dpadDownWas) {
                selectedSlot++;
                if (selectedSlot > MAX_SLOTS) selectedSlot = 1;
                dpadDownWas = true;
            } else if (!gamepad1.dpad_down) {
                dpadDownWas = false;
            }

            if (gamepad1.x && !xWas) {
                voltageCompEnabled = !voltageCompEnabled;
                xWas = true;
            } else if (!gamepad1.x) {
                xWas = false;
            }

            // Parse the recording whenever the selection changes (and on the first pass).
            if (selectedSlot != loadedSlot) {
                resetLoadedData();
                loadRecordingData();
                loadedSlot = selectedSlot;
            }

            drawSelectionUI();
            doSleep(60);
        }
    }

    private void drawSelectionUI() {
        telemetry.clear();
        telemetry.addLine("╔══════════════════════════════════════╗");
        telemetry.addLine("║    UNIVERSAL AUTO PLAYBACK          ║");
        telemetry.addLine("╠══════════════════════════════════════╣");
        telemetry.addLine("  D-Pad UP/DOWN: Select slot");
        telemetry.addLine("  X:             Toggle voltage comp");
        telemetry.addLine("  START:         Begin playback");
        telemetry.addLine("╠══════════════════════════════════════╣");
        telemetry.addData("Voltage comp (X)", voltageCompEnabled ? "ON" : "OFF");
        telemetry.addLine();

        for (int i = 1; i <= MAX_SLOTS; i++) {
            String prefix = (i == selectedSlot) ? ">" : " ";
            File f = new File(STORAGE_DIR + "slot_" + pad(i) + ".csv");
            if (f.exists() && f.length() > 0) {
                telemetry.addData(prefix + " Slot " + i, "Available (" + f.length() / 1024 + " KB)");
            } else {
                telemetry.addData(prefix + " Slot " + i, "-- EMPTY --");
            }
        }
        telemetry.addLine("╚══════════════════════════════════════╝");
        telemetry.addLine();

        if (loadWarning != null) telemetry.addData("WARNING", loadWarning);
        if (loadError != null) {
            telemetry.addData("Slot " + selectedSlot, "ERROR: " + loadError);
        } else if (frameData.isEmpty()) {
            telemetry.addData("Slot " + selectedSlot, "-- EMPTY --");
        } else {
            telemetry.addData("Loaded", "Slot " + selectedSlot);
            telemetry.addData("Frames", frameData.size());
            telemetry.addData("Devices", deviceDefs.size());
            long duration = frameTimestamps.get(frameTimestamps.size() - 1)[0];
            telemetry.addData("Duration", duration / 1000.0 + "s");
            String comp = !voltageColumnPresent ? "not in file"
                    : (playbackVoltageSensor == null ? "no sensor here"
                    : (voltageCompEnabled ? "ON" : "OFF (X)"));
            telemetry.addData("Voltage comp", comp);
            telemetry.addLine("Press START to begin playback");
        }
        telemetry.update();
    }

    // ========================
    // LOAD RECORDING
    // ========================

    private void resetLoadedData() {
        deviceDefs.clear();
        frameTimestamps.clear();
        frameData.clear();
        frameVbat.clear();
        motors.clear();
        servos.clear();
        crServos.clear();
        voltageColumnPresent = false;
        loadError = null;
        loadWarning = null;
    }

    /** Parse a slot's recording into deviceDefs + frame data. No hardware is touched here. */
    private void loadRecordingData() {
        File file = new File(STORAGE_DIR + "slot_" + pad(selectedSlot) + ".csv");
        if (!file.exists()) return; // UI renders this slot as EMPTY

        try (BufferedReader reader = new BufferedReader(new FileReader(file))) {
            String first = reader.readLine();
            if (first == null) {
                loadError = "empty file";
                return;
            }
            if (first.startsWith("#FORMAT")) {
                loadFormat2(reader, first);
            } else if (first.startsWith("#DEVICES")) {
                parseHeaderV1(first);
                loadDataRowsV1(reader);
            } else {
                loadError = "invalid format";
            }
        } catch (IOException e) {
            loadError = "load error: " + e.getMessage();
        }
    }

    private void cachePlaybackVoltageSensor() {
        try {
            if (hardwareMap.voltageSensor.size() > 0) {
                playbackVoltageSensor = hardwareMap.voltageSensor.iterator().next();
            }
        } catch (Exception ignored) {
            playbackVoltageSensor = null;
        }
    }

    // ---- Format 2 (current) ----

    private void loadFormat2(BufferedReader reader, String formatLine) throws IOException {
        warnIfNewerFormat(formatLine);

        boolean columnsChecked = false;
        int declaredDataCols = -1;
        String line;
        while ((line = reader.readLine()) != null) {
            if (line.isEmpty()) continue;

            if (line.charAt(0) == '#') {
                if (line.startsWith("#META")) {
                    parseMeta(line);
                } else if (line.startsWith("#DEV")) {
                    parseDev(line);
                } else if (line.startsWith("#COLUMNS")) {
                    declaredDataCols = line.split("\t").length - 1;
                }
                // unknown #-line → skip (forward compatibility)
            } else {
                if (!columnsChecked) {
                    int expected = 1 + (voltageColumnPresent ? 1 : 0) + deviceDefs.size();
                    if (declaredDataCols != -1 && declaredDataCols != expected) {
                        loadError = "header/column mismatch (corrupt file)";
                        return;
                    }
                    columnsChecked = true;
                }
                parseDataRowV2(line);
            }
        }
        // Device reconstruction (initDevices) happens once, at START, from runOpMode().
    }

    private void warnIfNewerFormat(String formatLine) {
        try {
            int v = Integer.parseInt(formatLine.substring(7).trim());
            if (v > FORMAT_VERSION) {
                loadWarning = "recording is v" + v + " (newer than this playback) — update it";
            }
        } catch (Exception ignored) {
        }
    }

    private void parseMeta(String line) {
        String[] tokens = line.split("\t");
        voltageColumnPresent = "1".equals(getField(tokens, "voltageComp"));
    }

    private void parseDev(String line) {
        String[] tokens = line.split("\t");
        String cls = getField(tokens, "class");
        String name = getField(tokens, "name");
        if (cls == null || name == null) return;

        DeviceDef def = new DeviceDef();
        def.name = unescapeField(name);

        if ("motor".equals(cls)) {
            def.type = DeviceType.MOTOR;
            def.kind = "velocity".equals(getField(tokens, "kind"))
                    ? ChannelKind.VELOCITY : ChannelKind.POWER;
            def.motorDir = "REV".equals(getField(tokens, "dir"))
                    ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD;
            def.motorZbp = "FLOAT".equals(getField(tokens, "zpb"))
                    ? DcMotor.ZeroPowerBehavior.FLOAT : DcMotor.ZeroPowerBehavior.BRAKE;
            String pidf = getField(tokens, "pidf");
            if (pidf != null) def.pidf = parsePidf(pidf);
            def.wasRunToPosition = "wasRunToPosition".equals(getField(tokens, "note"));
        } else if ("servo".equals(cls)) {
            def.type = DeviceType.SERVO;
        } else if ("crservo".equals(cls)) {
            def.type = DeviceType.CRSERVO;
        } else {
            return; // unknown class → skip
        }
        deviceDefs.add(def);
    }

    private void parseDataRowV2(String line) {
        String[] parts = line.split(",");
        int expected = 1 + (voltageColumnPresent ? 1 : 0) + deviceDefs.size();
        if (parts.length != expected) return;
        try {
            long[] ts = new long[]{Long.parseLong(parts[0])};
            int off = 1;
            double vb = -1;
            if (voltageColumnPresent) {
                vb = Double.parseDouble(parts[1]);
                off = 2;
            }
            double[] data = new double[deviceDefs.size()];
            for (int i = 0; i < deviceDefs.size(); i++) {
                double v = Double.parseDouble(parts[off + i]);
                data[i] = Double.isFinite(v) ? v : 0.0; // clamp the offending column, keep the frame
            }
            if (!Double.isFinite(vb)) vb = -1; // bad vbat → no comp for this frame
            frameTimestamps.add(ts);
            frameData.add(data);
            if (voltageColumnPresent) frameVbat.add(vb);
        } catch (NumberFormatException ignored) {
        }
    }

    // ---- Format 1 (legacy #DEVICES) ----

    private void parseHeaderV1(String header) {
        String content = header.substring(8).trim();
        String[] tokens = content.split(" ");

        for (String token : tokens) {
            if (token.isEmpty()) continue;
            DeviceDef def = new DeviceDef();

            if (token.startsWith("motor:")) {
                def.type = DeviceType.MOTOR;
                def.kind = ChannelKind.POWER; // v1 had no velocity channels
                String[] parts = token.substring(6).split(",");
                def.name = parts[0];
                if (parts.length >= 2) {
                    def.motorDir = "REV".equals(parts[1])
                            ? DcMotorSimple.Direction.REVERSE
                            : DcMotorSimple.Direction.FORWARD;
                }
                if (parts.length >= 3) {
                    def.motorZbp = "FLOAT".equals(parts[2])
                            ? DcMotor.ZeroPowerBehavior.FLOAT
                            : DcMotor.ZeroPowerBehavior.BRAKE;
                }
            } else if (token.startsWith("servo:")) {
                def.type = DeviceType.SERVO;
                def.name = token.substring(6);
            } else if (token.startsWith("crservo:")) {
                def.type = DeviceType.CRSERVO;
                def.name = token.substring(8);
            } else {
                continue;
            }
            deviceDefs.add(def);
        }
    }

    private void loadDataRowsV1(BufferedReader reader) throws IOException {
        String line;
        while ((line = reader.readLine()) != null) {
            String[] parts = line.split(",");
            if (parts.length != deviceDefs.size() + 1) continue;
            try {
                long[] ts = new long[]{Long.parseLong(parts[0])};
                double[] data = new double[deviceDefs.size()];
                for (int i = 0; i < deviceDefs.size(); i++) {
                    double v = Double.parseDouble(parts[i + 1]);
                    data[i] = Double.isFinite(v) ? v : 0.0; // clamp the offending column, keep the frame
                }
                frameTimestamps.add(ts);
                frameData.add(data);
            } catch (NumberFormatException ignored) {
            }
        }
    }

    // ---- Device reconstruction ----

    private void initDevices() {
        for (DeviceDef def : deviceDefs) {
            try {
                switch (def.type) {
                    case MOTOR: {
                        DcMotorEx motor = hardwareMap.get(DcMotorEx.class, def.name);
                        motor.setDirection(def.motorDir);
                        motor.setZeroPowerBehavior(def.motorZbp);
                        if (def.kind == ChannelKind.VELOCITY) {
                            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                            if (def.pidf != null) {
                                try {
                                    motor.setVelocityPIDFCoefficients(
                                            def.pidf[0], def.pidf[1], def.pidf[2], def.pidf[3]);
                                } catch (Exception e) {
                                    telemetry.addData("WARN: PIDF restore failed", def.name);
                                }
                            }
                            motor.setVelocity(0);
                        } else {
                            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                            motor.setPower(0);
                            if (def.wasRunToPosition) {
                                telemetry.addData(
                                        "WARN: RUN_TO_POSITION replayed as power", def.name);
                            }
                        }
                        motors.add(motor);
                        servos.add(null);
                        crServos.add(null);
                        break;
                    }
                    case SERVO: {
                        Servo servo = hardwareMap.get(Servo.class, def.name);
                        servos.add(servo);
                        motors.add(null);
                        crServos.add(null);
                        break;
                    }
                    case CRSERVO: {
                        CRServo crServo = hardwareMap.get(CRServo.class, def.name);
                        crServos.add(crServo);
                        motors.add(null);
                        servos.add(null);
                        break;
                    }
                }
            } catch (Exception e) {
                motors.add(null);
                servos.add(null);
                crServos.add(null);
                telemetry.addData("WARNING: Device not found", def.name);
            }
        }
    }

    // ========================
    // PLAYBACK WITH INTERPOLATION
    // ========================

    private void executePlayback() {
        if (frameData.isEmpty()) {
            telemetry.addLine("No frames to play back.");
            telemetry.update();
            return;
        }

        long playbackStart = System.currentTimeMillis();
        int totalFrames = frameData.size();
        long totalDuration = frameTimestamps.get(totalFrames - 1)[0];
        long safetyTimeout = Math.max(totalDuration + 5000, SAFETY_TIMEOUT_MS);

        telemetry.addLine("Playback started...");
        telemetry.addData("Frames", totalFrames);
        telemetry.addData("Duration", totalDuration / 1000.0 + "s");
        telemetry.update();

        int prevIdx = 0;

        while (opModeIsActive()) {
            long elapsed = System.currentTimeMillis() - playbackStart;

            if (elapsed > safetyTimeout) {
                telemetry.addLine("SAFETY TIMEOUT — stopping playback");
                break;
            }

            // Advance prevIdx to the last frame with ts <= elapsed
            while (prevIdx + 1 < totalFrames && frameTimestamps.get(prevIdx + 1)[0] <= elapsed) {
                prevIdx++;
            }

            int nextIdx = prevIdx + 1;
            double currentVbat = readPlaybackVoltage();

            if (nextIdx >= totalFrames) {
                // Past end — hold last frame, then exit
                applyFrameHold(totalFrames - 1, currentVbat);
                break;
            }

            double prevTs = frameTimestamps.get(prevIdx)[0];
            double nextTs = frameTimestamps.get(nextIdx)[0];
            double denom = nextTs - prevTs;
            double t = (denom <= 0) ? 1.0 : (elapsed - prevTs) / denom; // guard 0/0 = NaN
            if (t < 0) t = 0;
            if (t > 1) t = 1;

            double[] prevData = frameData.get(prevIdx);
            double[] nextData = frameData.get(nextIdx);

            double recordedVbat = -1;
            if (voltageColumnPresent) {
                double pv = frameVbat.get(prevIdx);
                double nv = frameVbat.get(nextIdx);
                recordedVbat = pv + t * (nv - pv);
            }

            for (int col = 0; col < deviceDefs.size(); col++) {
                double value = prevData[col] + t * (nextData[col] - prevData[col]);
                applyDeviceValue(col, value, recordedVbat, currentVbat);
            }

            // Telemetry every ~250ms
            if (prevIdx % 50 == 0 && prevIdx != 0) {
                int progress = (int) (100.0 * prevIdx / totalFrames);
                telemetry.addData("Progress", progress + "%  (" + prevIdx + "/" + totalFrames + ")");
                telemetry.addData("Time", elapsed / 1000.0 + "s / " + totalDuration / 1000.0 + "s");
                telemetry.update();
            }

            doSleep(LOOP_SLEEP_MS);
        }

        stopAll();
        telemetry.addLine("Playback complete.");
        telemetry.update();
    }

    private void applyFrameHold(int frameIdx, double currentVbat) {
        double[] data = frameData.get(frameIdx);
        double recordedVbat = voltageColumnPresent ? frameVbat.get(frameIdx) : -1;
        for (int col = 0; col < deviceDefs.size(); col++) {
            applyDeviceValue(col, data[col], recordedVbat, currentVbat);
        }
    }

    private void applyDeviceValue(int col, double value, double recordedVbat, double currentVbat) {
        if (col >= deviceDefs.size()) return;
        if (!Double.isFinite(value)) value = 0; // never command NaN/Infinity to hardware
        DeviceDef def = deviceDefs.get(col);

        try {
            switch (def.type) {
                case MOTOR: {
                    DcMotorEx m = motors.get(col);
                    if (m == null) break;
                    if (def.kind == ChannelKind.VELOCITY) {
                        m.setVelocity(value); // ticks/sec — closed loop, voltage-independent
                    } else {
                        m.setPower(scaleForVoltage(value, recordedVbat, currentVbat));
                    }
                    break;
                }
                case SERVO:
                    if (servos.get(col) != null) servos.get(col).setPosition(value);
                    break;
                case CRSERVO:
                    if (crServos.get(col) != null) crServos.get(col).setPower(value);
                    break;
            }
        } catch (Exception ignored) {
        }
    }

    /** Scale an open-loop power command for the playback-day battery voltage. */
    private double scaleForVoltage(double power, double recordedVbat, double currentVbat) {
        if (!voltageCompActive()) return clamp(power, -1.0, 1.0);
        if (recordedVbat <= 0 || currentVbat <= 0.1) return clamp(power, -1.0, 1.0);
        return clamp(power * (recordedVbat / currentVbat), -1.0, 1.0);
    }

    private boolean voltageCompActive() {
        return voltageColumnPresent && voltageCompEnabled && playbackVoltageSensor != null;
    }

    private double readPlaybackVoltage() {
        if (playbackVoltageSensor == null) return -1;
        try {
            return playbackVoltageSensor.getVoltage();
        } catch (Exception e) {
            return -1;
        }
    }

    private void stopAll() {
        for (int i = 0; i < motors.size(); i++) {
            DcMotorEx m = motors.get(i);
            if (m == null) continue;
            try {
                if (deviceDefs.get(i).kind == ChannelKind.VELOCITY) {
                    m.setVelocity(0);
                } else {
                    m.setPower(0);
                }
            } catch (Exception ignored) {
            }
        }
        for (CRServo c : crServos) {
            if (c != null) {
                try {
                    c.setPower(0);
                } catch (Exception ignored) {
                }
            }
        }
    }

    // ========================
    // UTILITY
    // ========================

    private static double clamp(double v, double lo, double hi) {
        return v < lo ? lo : (v > hi ? hi : v);
    }

    /** Reads the value of {@code key=...} from TAB-split header tokens, or null. */
    private static String getField(String[] tokens, String key) {
        String prefix = key + "=";
        for (String t : tokens) {
            if (t.startsWith(prefix)) return t.substring(prefix.length());
        }
        return null;
    }

    private static double[] parsePidf(String s) {
        String[] p = s.split(",");
        if (p.length != 4) return null;
        try {
            return new double[]{
                    Double.parseDouble(p[0]), Double.parseDouble(p[1]),
                    Double.parseDouble(p[2]), Double.parseDouble(p[3])
            };
        } catch (NumberFormatException e) {
            return null;
        }
    }

    private static String unescapeField(String s) {
        if (s.indexOf('\\') < 0) return s;
        StringBuilder out = new StringBuilder();
        for (int i = 0; i < s.length(); i++) {
            char c = s.charAt(i);
            if (c == '\\' && i + 1 < s.length()) {
                char n = s.charAt(++i);
                switch (n) {
                    case 't': out.append('\t'); break;
                    case 'n': out.append('\n'); break;
                    case '\\': out.append('\\'); break;
                    default: out.append(n);
                }
            } else {
                out.append(c);
            }
        }
        return out.toString();
    }

    private String pad(int num) {
        return num < 10 ? "0" + num : String.valueOf(num);
    }

    private void doSleep(long ms) {
        try {
            Thread.sleep(ms);
        } catch (InterruptedException ignored) {
        }
    }
}
