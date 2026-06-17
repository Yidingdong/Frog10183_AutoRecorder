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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileOutputStream;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.io.OutputStreamWriter;
import java.nio.charset.StandardCharsets;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Date;
import java.util.IdentityHashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;

/**
 * Universal Auto Recorder — zero-config recording for ANY FTC robot.
 *
 * <p>Automatically discovers all motors, servos, and CR servos from the HardwareMap.
 * Stores up to 10 recordings with an init-time slot selection UI. Recorded device values
 * are replayed with linear interpolation by {@link UniversalAutoPlayback}.</p>
 *
 * <p>Faithful by design: each motor is recorded according to its <b>control mode</b> —
 * a closed-loop velocity motor (e.g. a flywheel using {@code RUN_USING_ENCODER} +
 * {@code setVelocity}) is recorded and replayed as a velocity target in ticks/sec
 * (proportional to RPM), not raw power, so it holds speed regardless of battery voltage.
 * Open-loop power channels are additionally battery-voltage compensated on playback.</p>
 *
 * <h3>Integration (import + field + 4 calls in your LinearOpMode):</h3>
 * <pre>{@code
 * import org.firstinspires.ftc.teamcode.universal.UniversalAutoRecorder;
 *
 * private UniversalAutoRecorder recorder;
 *
 * // In runOpMode(), after hardware init, before waitForStart() (order-independent —
 * // motor modes are re-sampled when recording starts):
 * recorder = new UniversalAutoRecorder(hardwareMap, gamepad1, telemetry, this);
 * recorder.waitForSlotSelection();
 *
 * // In your while (opModeIsActive()) loop:
 * recorder.capture();
 * // Optional: recorder.showTelemetry();
 *
 * // After your main loop (reached on STOP):
 * recorder.close();
 * }</pre>
 *
 * <p>Passing {@code this} (your LinearOpMode) is recommended: it lets the recorder exit
 * cleanly if STOP is pressed during slot selection and accept the Driver Station Play
 * button to begin. The legacy 3-argument constructor still works, but it cannot detect the
 * Driver Station Play button — on that path you must press gamepad A to start recording.</p>
 *
 * <p>Playback: run the standalone {@link UniversalAutoPlayback} OpMode in autonomous.</p>
 */
public class UniversalAutoRecorder {

    /** File format version written by this recorder. Must match UniversalAutoPlayback. */
    static final int FORMAT_VERSION = 2;

    private static final String STORAGE_DIR = "/sdcard/FIRST/autorecorder/";
    private static final String INDEX_FILE = STORAGE_DIR + "index.json";
    private static final int MAX_SLOTS = 10;
    private static final long CAPTURE_INTERVAL_MS = 20;

    /** How a motor's value is recorded/replayed. Servos/CRServos are handled separately. */
    private enum ChannelKind { POWER, VELOCITY }

    private final HardwareMap hardwareMap;
    private final Gamepad gamepad;
    private final Telemetry telemetry;
    private final LinearOpMode opMode; // may be null (legacy constructor)

    // Discovered devices (ordered — column 0 = timestamp, optional column 1 = vbat)
    private final List<String> motorNames = new ArrayList<>();
    private final List<DcMotorEx> motors = new ArrayList<>();
    private final List<DcMotorSimple.Direction> motorDirections = new ArrayList<>();
    private final List<DcMotor.ZeroPowerBehavior> motorBehaviors = new ArrayList<>();
    private final List<ChannelKind> motorKinds = new ArrayList<>();
    private final List<PIDFCoefficients> motorPidfs = new ArrayList<>(); // null unless velocity
    private final List<Boolean> motorWasRunToPosition = new ArrayList<>();
    private final List<String> skippedMotorNames = new ArrayList<>(); // dcMotors that aren't DcMotorEx
    private final List<String> skippedAliasNames = new ArrayList<>(); // extra names pointing at an already-found device

    private final List<String> servoNames = new ArrayList<>();
    private final List<Servo> servos = new ArrayList<>();

    private final List<String> crServoNames = new ArrayList<>();
    private final List<CRServo> crServos = new ArrayList<>();

    // Battery voltage compensation
    private VoltageSensor voltageSensor;        // cached, null if none
    private boolean voltageCompRequested = true;
    private boolean voltageCompEnabled = false; // requested AND a sensor exists
    private double lastGoodVbat = 12.0;         // fallback so a transient bad read can't poison a frame

    // Slot state (1-indexed)
    private final SlotInfo[] slots = new SlotInfo[MAX_SLOTS + 1];
    private int selectedSlot = 1;
    private boolean recording = false;
    private boolean selectionConfirmed = false;
    private boolean aborted = false;

    private long recordingStartTime;
    private long lastCaptureTime;
    private int frameCount;
    private List<String> recordingRows;

    // Gamepad debounce
    private boolean dpadUpWas;
    private boolean dpadDownWas;
    private boolean bWas;
    private boolean aWas;

    private static class SlotInfo {
        boolean occupied;
        String date;
        long durationMs;
        int frames;
        int deviceCount;
    }

    /**
     * Legacy constructor — no OpMode lifecycle awareness. On this path the Driver Station
     * Play button cannot confirm the slot (there is no OpMode to observe), so you MUST press
     * gamepad A to begin; pressing DS Play just leaves it on the selection screen. STOP still
     * exits via thread interrupt. Prefer the 4-arg version.
     *
     * @deprecated use {@link #UniversalAutoRecorder(HardwareMap, Gamepad, Telemetry, LinearOpMode)}.
     */
    @Deprecated
    public UniversalAutoRecorder(HardwareMap hardwareMap, Gamepad gamepad, Telemetry telemetry) {
        this(hardwareMap, gamepad, telemetry, null);
    }

    /**
     * Recommended constructor. Pass your LinearOpMode ({@code this}) so the recorder can
     * exit cleanly on STOP and accept the Driver Station Play button during slot selection.
     */
    public UniversalAutoRecorder(HardwareMap hardwareMap, Gamepad gamepad, Telemetry telemetry,
                                 LinearOpMode opMode) {
        this.hardwareMap = hardwareMap;
        this.gamepad = gamepad;
        this.telemetry = telemetry;
        this.opMode = opMode;
        for (int i = 1; i <= MAX_SLOTS; i++) {
            slots[i] = new SlotInfo();
        }
        discoverDevices();
        cacheVoltageSensor();
        loadIndex();
    }

    // ========================
    // DEVICE DISCOVERY
    // ========================

    private void discoverDevices() {
        // One column per *physical* actuator. If a config binds the same device under
        // several names (aliases), keep the first and skip the rest — otherwise it would
        // be recorded and driven twice. Not reachable via the normal Driver Station config
        // UI (one name per port), but hand-edited config XML can do it.
        IdentityHashMap<Object, String> seen = new IdentityHashMap<>();
        for (Map.Entry<String, DcMotor> entry : hardwareMap.dcMotor.entrySet()) {
            DcMotor motor = entry.getValue();
            if (!(motor instanceof DcMotorEx)) {
                skippedMotorNames.add(entry.getKey()); // not DcMotorEx — cannot record (very rare)
                continue;
            }
            if (seen.containsKey(motor)) {
                skippedAliasNames.add(entry.getKey());
                continue;
            }
            seen.put(motor, entry.getKey());
            motorNames.add(entry.getKey());
            motors.add((DcMotorEx) motor);
        }
        for (Map.Entry<String, Servo> entry : hardwareMap.servo.entrySet()) {
            Servo servo = entry.getValue();
            if (seen.containsKey(servo)) {
                skippedAliasNames.add(entry.getKey());
                continue;
            }
            seen.put(servo, entry.getKey());
            servoNames.add(entry.getKey());
            servos.add(servo);
        }
        for (Map.Entry<String, CRServo> entry : hardwareMap.crservo.entrySet()) {
            CRServo crServo = entry.getValue();
            if (seen.containsKey(crServo)) {
                skippedAliasNames.add(entry.getKey());
                continue;
            }
            seen.put(crServo, entry.getKey());
            crServoNames.add(entry.getKey());
            crServos.add(crServo);
        }
        snapshotMotorChannels(); // initial classification for the slot-screen hint
    }

    /**
     * Sample each motor's control mode, PIDF, direction and zero-power behavior.
     * Called at construction (for the slot-screen hint) and again the moment recording
     * starts — so the recording reflects the modes in effect at that point, and you can
     * construct the recorder anywhere in init regardless of when subsystems set their modes.
     */
    private void snapshotMotorChannels() {
        motorDirections.clear();
        motorBehaviors.clear();
        motorKinds.clear();
        motorPidfs.clear();
        motorWasRunToPosition.clear();
        for (DcMotorEx motorEx : motors) {
            // Sample defensively into locals first, then add all 5 lists together, so a
            // (very unlikely) SDK getter exception can never desync the parallel lists or
            // propagate out of capture(). These getters are cache reads and rarely throw.
            DcMotorSimple.Direction dir = DcMotorSimple.Direction.FORWARD;
            DcMotor.ZeroPowerBehavior zpb = DcMotor.ZeroPowerBehavior.BRAKE;
            DcMotor.RunMode mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER;
            PIDFCoefficients pidf = null;
            try {
                dir = motorEx.getDirection();
                zpb = motorEx.getZeroPowerBehavior();
                mode = motorEx.getMode();
                if (mode == DcMotor.RunMode.RUN_USING_ENCODER) {
                    pidf = motorEx.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
                }
            } catch (Exception ignored) {
                // keep defaults / partial reads; playback falls back to hub-default PIDF
            }
            boolean velocity = (mode == DcMotor.RunMode.RUN_USING_ENCODER);
            motorDirections.add(dir);
            motorBehaviors.add(zpb);
            motorKinds.add(velocity ? ChannelKind.VELOCITY : ChannelKind.POWER);
            motorPidfs.add(velocity ? pidf : null);
            motorWasRunToPosition.add(mode == DcMotor.RunMode.RUN_TO_POSITION);
        }
    }

    private void cacheVoltageSensor() {
        try {
            if (hardwareMap.voltageSensor.size() > 0) {
                voltageSensor = hardwareMap.voltageSensor.iterator().next();
            }
        } catch (Exception ignored) {
            voltageSensor = null;
        }
        voltageCompEnabled = voltageCompRequested && (voltageSensor != null);
    }

    /**
     * Enable/disable per-frame battery-voltage compensation for open-loop power channels.
     * Default ON when a voltage sensor exists. Call before the first {@link #capture()}.
     */
    public void setVoltageCompensation(boolean enabled) {
        if (recording) return; // too late — the header is already written
        this.voltageCompRequested = enabled;
        this.voltageCompEnabled = enabled && (voltageSensor != null);
    }

    // ========================
    // INDEX FILE
    // ========================

    private void loadIndex() {
        File file = new File(INDEX_FILE);
        if (!file.exists()) return;
        try (BufferedReader reader = new BufferedReader(new FileReader(file))) {
            StringBuilder json = new StringBuilder();
            String line;
            while ((line = reader.readLine()) != null) {
                json.append(line);
            }
            jsonToSlots(json.toString());
        } catch (Exception ignored) {
        }
    }

    private void jsonToSlots(String json) {
        int pos = 0;
        while ((pos = json.indexOf("\"", pos)) != -1) {
            int keyStart = pos + 1;
            int keyEnd = json.indexOf("\"", keyStart);
            if (keyEnd == -1) break;
            int slotNum;
            try {
                slotNum = Integer.parseInt(json.substring(keyStart, keyEnd));
            } catch (NumberFormatException e) {
                pos = keyEnd + 1;
                continue;
            }
            if (slotNum < 1 || slotNum > MAX_SLOTS) {
                pos = keyEnd + 1;
                continue;
            }
            int objStart = json.indexOf("{", keyEnd);
            int objEnd = json.indexOf("}", objStart);
            if (objStart == -1 || objEnd == -1) break;

            String obj = json.substring(objStart + 1, objEnd);
            SlotInfo info = slots[slotNum];
            info.occupied = booleanFromJson(obj, "occupied");
            info.date = stringFromJson(obj, "date");
            info.durationMs = longFromJson(obj, "duration_ms");
            info.frames = intFromJson(obj, "frames");
            info.deviceCount = intFromJson(obj, "device_count");
            pos = objEnd + 1;
        }
    }

    private void saveIndex() {
        StringBuilder json = new StringBuilder("{");
        boolean firstSlot = true;
        for (int i = 1; i <= MAX_SLOTS; i++) {
            if (!firstSlot) json.append(",");
            firstSlot = false;
            SlotInfo s = slots[i];
            json.append("\"").append(i).append("\":{");
            json.append("\"occupied\":").append(s.occupied);
            if (s.occupied && s.date != null) {
                json.append(",\"date\":\"").append(escapeJson(s.date)).append("\"");
                json.append(",\"duration_ms\":").append(s.durationMs);
                json.append(",\"frames\":").append(s.frames);
                json.append(",\"device_count\":").append(s.deviceCount);
            }
            json.append("}");
        }
        json.append("}");

        try {
            File dir = new File(STORAGE_DIR);
            if (!dir.exists()) dir.mkdirs();
            // Write to a temp file then rename, so a crash mid-write can't corrupt index.json.
            File tmp = new File(INDEX_FILE + ".tmp");
            try (FileWriter writer = new FileWriter(tmp)) {
                writer.write(json.toString());
            }
            File dest = new File(INDEX_FILE);
            if (!tmp.renameTo(dest)) {
                dest.delete();
                tmp.renameTo(dest);
            }
        } catch (IOException ignored) {
        }
    }

    private static boolean booleanFromJson(String obj, String key) {
        int idx = obj.indexOf("\"" + key + "\":");
        if (idx == -1) return false;
        int valStart = idx + key.length() + 3;
        return obj.startsWith("true", valStart);
    }

    private static String stringFromJson(String obj, String key) {
        int idx = obj.indexOf("\"" + key + "\":\"");
        if (idx == -1) return "";
        int valStart = idx + key.length() + 4;
        int valEnd = obj.indexOf("\"", valStart);
        if (valEnd == -1) return "";
        return obj.substring(valStart, valEnd);
    }

    private static long longFromJson(String obj, String key) {
        int idx = obj.indexOf("\"" + key + "\":");
        if (idx == -1) return 0;
        int valStart = idx + key.length() + 3;
        int valEnd = valStart;
        while (valEnd < obj.length() && (Character.isDigit(obj.charAt(valEnd)) || obj.charAt(valEnd) == '-')) {
            valEnd++;
        }
        try {
            return Long.parseLong(obj.substring(valStart, valEnd));
        } catch (NumberFormatException e) {
            return 0;
        }
    }

    private static int intFromJson(String obj, String key) {
        return (int) longFromJson(obj, key);
    }

    private static String escapeJson(String s) {
        return s.replace("\\", "\\\\").replace("\"", "\\\"");
    }

    // ========================
    // SLOT SELECTION UI
    // ========================

    /**
     * Blocking init-time slot selection UI.
     * Shows discovered devices, slot metadata, and lets the user choose which slot to record.
     * D-Pad Up/Down selects, B deletes.
     * Press A (or the Driver Station Play button) to confirm — recording begins automatically
     * on the first {@link #capture()} call. Exits immediately if STOP is requested.
     */
    public void waitForSlotSelection() {
        while (!selectionConfirmed) {
            // Exit cleanly on STOP. The interrupt check also rescues the legacy 3-arg
            // constructor (opMode == null), where isStopRequested() is always false:
            // the SDK interrupts this thread on STOP, and sleep() re-asserts that flag.
            if (isStopRequested() || Thread.currentThread().isInterrupted()) {
                aborted = true;
                return;
            }
            // Driver Station Play button also confirms the selection.
            if (isStarted()) {
                selectionConfirmed = true;
                break;
            }

            // A — confirm and begin
            if (gamepad.a && !aWas) {
                aWas = true;
                selectionConfirmed = true;
                break;
            } else if (!gamepad.a) {
                aWas = false;
            }

            // D-Pad up — select previous slot
            if (gamepad.dpad_up && !dpadUpWas) {
                selectedSlot--;
                if (selectedSlot < 1) selectedSlot = MAX_SLOTS;
                dpadUpWas = true;
            } else if (!gamepad.dpad_up) {
                dpadUpWas = false;
            }

            // D-Pad down — select next slot
            if (gamepad.dpad_down && !dpadDownWas) {
                selectedSlot++;
                if (selectedSlot > MAX_SLOTS) selectedSlot = 1;
                dpadDownWas = true;
            } else if (!gamepad.dpad_down) {
                dpadDownWas = false;
            }

            // B — delete selected slot
            if (gamepad.b && !bWas) {
                bWas = true;
                deleteSlot(selectedSlot);
            } else if (!gamepad.b) {
                bWas = false;
            }

            drawSlotSelectionUI();
            sleep(60);
        }
        if (!aborted) drawConfirmation();
    }

    private boolean isStopRequested() {
        return opMode != null && opMode.isStopRequested();
    }

    private boolean isStarted() {
        return opMode != null && opMode.isStarted();
    }

    private void deleteSlot(int slot) {
        SlotInfo info = slots[slot];
        info.occupied = false;
        info.date = null;
        info.durationMs = 0;
        info.frames = 0;
        info.deviceCount = 0;

        String csvFile = STORAGE_DIR + "slot_" + pad(slot) + ".csv";
        File f = new File(csvFile);
        if (f.exists()) f.delete();
        saveIndex();
    }

    private String pad(int num) {
        return num < 10 ? "0" + num : String.valueOf(num);
    }

    /** A slot is "occupied" iff its CSV exists and is non-empty — the file is the truth. */
    private boolean slotHasFile(int slot) {
        File f = new File(STORAGE_DIR + "slot_" + pad(slot) + ".csv");
        return f.exists() && f.length() > 0;
    }

    private void drawSlotSelectionUI() {
        telemetry.clear();
        telemetry.addLine("╔══════════════════════════════════════╗");
        telemetry.addLine("║     UNIVERSAL AUTO RECORDER         ║");
        telemetry.addLine("╠══════════════════════════════════════╣");

        // Device inventory
        int totalDevices = motorNames.size() + servoNames.size() + crServoNames.size();
        telemetry.addData("Devices found", "Motors:" + motorNames.size()
                + "  Servos:" + servoNames.size()
                + "  CRServos:" + crServoNames.size()
                + "  Total:" + totalDevices);
        telemetry.addData("Voltage comp", voltageCompEnabled ? "ON" : "OFF (no sensor)");

        if (totalDevices == 0) {
            telemetry.addLine("WARNING: No devices discovered!");
            telemetry.addLine("Check your hardware config.");
        } else {
            StringBuilder deviceList = new StringBuilder();
            for (int i = 0; i < motorNames.size(); i++) {
                deviceList.append(motorKinds.get(i) == ChannelKind.VELOCITY ? " V:" : " M:")
                        .append(motorNames.get(i));
            }
            for (String n : servoNames) deviceList.append(" S:").append(n);
            for (String n : crServoNames) deviceList.append(" C:").append(n);
            telemetry.addData("Devices", deviceList.toString().trim());
            if (!motorNames.isEmpty()) {
                telemetry.addLine("  V=velocity M=power (re-checked when recording starts)");
            }
        }
        if (!skippedMotorNames.isEmpty()) {
            telemetry.addLine("  SKIPPED (not DcMotorEx): " + String.join(", ", skippedMotorNames));
        }
        if (!skippedAliasNames.isEmpty()) {
            telemetry.addLine("  SKIPPED (duplicate of another name): " + String.join(", ", skippedAliasNames));
        }

        telemetry.addLine("╠══════════════════════════════════════╣");
        telemetry.addLine("  D-Pad UP/DOWN: Select slot");
        telemetry.addLine("  B:             Delete slot");
        telemetry.addLine(opMode != null
                ? "  A / DS PLAY:   Begin recording"
                : "  A:             Begin recording (DS PLAY won't confirm)");
        telemetry.addLine("╠══════════════════════════════════════╣");

        // Slot list — the CSV file is the source of truth; index.json metadata is decoration
        // (so a copied-in recording shows as Available, and a desynced index can't hide a slot).
        for (int i = 1; i <= MAX_SLOTS; i++) {
            SlotInfo s = slots[i];
            String prefix = (i == selectedSlot) ? ">" : " ";
            if (slotHasFile(i)) {
                String meta = (s.occupied && s.date != null)
                        ? s.date + "  " + (s.durationMs / 1000.0) + "s  " + s.frames + "f"
                        : "Available";
                telemetry.addData(prefix + " Slot " + i, meta);
            } else {
                telemetry.addData(prefix + " Slot " + i, "-- EMPTY --");
            }
        }

        telemetry.addLine("╚══════════════════════════════════════╝");
        telemetry.addLine();
        telemetry.addData("Selected slot", selectedSlot);
        if (slotHasFile(selectedSlot)) {
            telemetry.addLine("WARNING: This slot WILL be overwritten!");
        }
        telemetry.update();
    }

    private void drawConfirmation() {
        telemetry.clear();
        telemetry.addLine("╔══════════════════════════════════════╗");
        telemetry.addLine("║     RECORDING TO SLOT " + selectedSlot + "             ║");
        telemetry.addLine("╚══════════════════════════════════════╝");
        telemetry.addLine();
        telemetry.addData("Motors", motorNames.size());
        telemetry.addData("Servos", servoNames.size());
        telemetry.addData("CR Servos", crServoNames.size());
        telemetry.addData("Voltage comp", voltageCompEnabled ? "ON" : "OFF");
        telemetry.addLine("(Motor modes are sampled when recording starts)");

        if (slotHasFile(selectedSlot)) {
            telemetry.addLine();
            telemetry.addLine("Previous recording will be overwritten.");
        }
        telemetry.addLine();
        telemetry.addLine("Recording begins when OpMode starts.");
        telemetry.update();
    }

    // ========================
    // RECORDING
    // ========================

    /**
     * Start recording. Called internally on the first {@link #capture()}.
     * Builds the file header and prepares the buffer.
     */
    private void startRecording() {
        recording = true;
        // Re-sample modes/PIDF/direction NOW, so the recording is correct even if the
        // recorder was constructed before subsystems set their motor modes.
        snapshotMotorChannels();
        recordingStartTime = System.currentTimeMillis();
        // Make the very first capture() record a frame at t≈0.
        lastCaptureTime = recordingStartTime - CAPTURE_INTERVAL_MS;
        frameCount = 0;
        recordingRows = new ArrayList<>();
        recordingRows.addAll(buildHeaderLines());
    }

    /** Builds the self-describing, versioned, TAB-delimited header (one device per line). */
    private List<String> buildHeaderLines() {
        List<String> lines = new ArrayList<>();
        lines.add("#FORMAT " + FORMAT_VERSION);

        String created = new SimpleDateFormat("yyyy-MM-dd HH:mm:ss", Locale.US).format(new Date());
        lines.add("#META\tcreated=" + created + "\tvoltageComp=" + (voltageCompEnabled ? "1" : "0"));

        int idx = 0;
        for (int i = 0; i < motorNames.size(); i++) {
            StringBuilder d = new StringBuilder("#DEV");
            d.append("\tidx=").append(idx++);
            d.append("\tclass=motor");
            d.append("\tname=").append(escapeField(motorNames.get(i)));
            boolean velocity = motorKinds.get(i) == ChannelKind.VELOCITY;
            d.append("\tkind=").append(velocity ? "velocity" : "power");
            d.append("\tdir=").append(directionToString(motorDirections.get(i)));
            d.append("\tzpb=").append(behaviorToString(motorBehaviors.get(i)));
            if (velocity && motorPidfs.get(i) != null) {
                PIDFCoefficients c = motorPidfs.get(i);
                d.append("\tpidf=")
                        .append(formatDouble(c.p, 5)).append(",")
                        .append(formatDouble(c.i, 5)).append(",")
                        .append(formatDouble(c.d, 5)).append(",")
                        .append(formatDouble(c.f, 5));
            }
            if (motorWasRunToPosition.get(i)) {
                d.append("\tnote=wasRunToPosition");
            }
            lines.add(d.toString());
        }
        for (String name : servoNames) {
            lines.add("#DEV\tidx=" + (idx++) + "\tclass=servo\tname=" + escapeField(name));
        }
        for (String name : crServoNames) {
            lines.add("#DEV\tidx=" + (idx++) + "\tclass=crservo\tname=" + escapeField(name));
        }

        StringBuilder cols = new StringBuilder("#COLUMNS\tt_ms");
        if (voltageCompEnabled) cols.append("\tvbat");
        for (int i = 0; i < idx; i++) cols.append("\td").append(i);
        lines.add(cols.toString());

        return lines;
    }

    private static String directionToString(DcMotorSimple.Direction dir) {
        return dir == DcMotorSimple.Direction.FORWARD ? "FWD" : "REV";
    }

    private static String behaviorToString(DcMotor.ZeroPowerBehavior zbp) {
        return zbp == DcMotor.ZeroPowerBehavior.BRAKE ? "BRAKE" : "FLOAT";
    }

    /** TAB and newline cannot occur in an FTC config name, but escape defensively anyway. */
    private static String escapeField(String s) {
        return s.replace("\\", "\\\\").replace("\t", "\\t").replace("\r", "").replace("\n", "\\n");
    }

    /**
     * Capture one frame of motor/servo states. Rate-limited to 50Hz internally.
     * Safe to call every loop — excess calls are silently ignored.
     * Auto-starts recording on the first call if not already started.
     */
    public void capture() {
        if (aborted) return;
        if (isStopRequested()) return;
        if (!recording) {
            startRecording();
        }

        long now = System.currentTimeMillis();
        if (now - lastCaptureTime < CAPTURE_INTERVAL_MS) return;
        lastCaptureTime = now;

        StringBuilder row = new StringBuilder();
        row.append(now - recordingStartTime);

        if (voltageCompEnabled) {
            double vbat = readBatteryVoltage();
            if (vbat > 0) {
                lastGoodVbat = vbat;
            } else {
                vbat = lastGoodVbat; // a transient I2C/read glitch must not poison the timeline
            }
            row.append(",").append(formatDouble(vbat, 2));
        }

        for (int i = 0; i < motors.size(); i++) {
            double value = (motorKinds.get(i) == ChannelKind.VELOCITY)
                    ? motors.get(i).getVelocity()   // measured ticks/sec
                    : motors.get(i).getPower();
            if (!Double.isFinite(value)) value = 0.0; // never write NaN/Inf — it would gap the frame on load
            row.append(",").append(formatDouble(value, 4));
        }
        for (Servo servo : servos) {
            double pos = servo.getPosition();
            if (!Double.isFinite(pos)) pos = 0.0;
            row.append(",").append(formatDouble(pos, 4));
        }
        for (CRServo crServo : crServos) {
            double crPower = crServo.getPower();
            if (!Double.isFinite(crPower)) crPower = 0.0;
            row.append(",").append(formatDouble(crPower, 4));
        }

        recordingRows.add(row.toString());
        frameCount++;
    }

    private double readBatteryVoltage() {
        if (voltageSensor == null) return -1;
        try {
            return voltageSensor.getVoltage();
        } catch (Exception e) {
            return -1;
        }
    }

    /**
     * Show recording status in telemetry.
     * Call this in your main loop if you want live recording feedback.
     */
    public void showTelemetry() {
        if (!recording) return;

        telemetry.addLine();
        telemetry.addLine("=== RECORDING ===");
        telemetry.addData("Slot", selectedSlot);
        telemetry.addData("Frames", frameCount);
        double elapsed = (System.currentTimeMillis() - recordingStartTime) / 1000.0;
        telemetry.addData("Duration", "%.1fs", elapsed);
        int deviceCount = motorNames.size() + servoNames.size() + crServoNames.size();
        telemetry.addData("Devices", deviceCount);
    }

    public boolean isRecording() {
        return recording;
    }

    // ========================
    // SAVE
    // ========================

    /**
     * Stop recording and save to disk. Call this when the OpMode stops.
     */
    public void close() {
        if (aborted) return;
        if (!recording) return;
        recording = false;

        File dest = new File(STORAGE_DIR + "slot_" + pad(selectedSlot) + ".csv");
        File tmp = new File(STORAGE_DIR + "slot_" + pad(selectedSlot) + ".csv.tmp");
        try {
            File dir = new File(STORAGE_DIR);
            if (!dir.exists() && !dir.mkdirs()) {
                throw new IOException("could not create " + STORAGE_DIR);
            }

            // Write to a temp file and atomically rename, so a STOP/crash mid-save can never
            // truncate or corrupt the slot — the previous recording stays intact until the new
            // one is fully on disk. Buffered write + one fsync keeps this well inside the SDK's
            // post-STOP watchdog window even for long recordings.
            try (FileOutputStream fos = new FileOutputStream(tmp);
                 BufferedWriter writer =
                         new BufferedWriter(new OutputStreamWriter(fos, StandardCharsets.UTF_8), 1 << 16)) {
                for (String row : recordingRows) {
                    writer.write(row);
                    writer.write('\n');
                }
                writer.flush();
                try {
                    fos.getFD().sync(); // force bytes to flash before the rename commit
                } catch (IOException ignored) {
                    // sync unsupported/failed — the rename is still safer than truncating in place
                }
            }
            if (!tmp.renameTo(dest)) {
                // Some filesystems won't rename onto an existing file — replace it.
                dest.delete();
                if (!tmp.renameTo(dest)) {
                    throw new IOException("commit (rename) failed");
                }
            }

            // Commit metadata only AFTER the CSV is durably in place, so index.json can never
            // claim a slot whose CSV write didn't finish.
            SlotInfo info = slots[selectedSlot];
            info.occupied = true;
            info.date = new SimpleDateFormat("yyyy-MM-dd HH:mm:ss", Locale.US).format(new Date());
            info.durationMs = frameCount > 0
                    ? parseTimestamp(recordingRows.get(recordingRows.size() - 1))
                    : 0;
            info.frames = frameCount;
            info.deviceCount = motorNames.size() + servoNames.size() + crServoNames.size();
            saveIndex();

            telemetry.addLine();
            telemetry.addLine("Recording saved!");
            telemetry.addData("Slot", selectedSlot);
            telemetry.addData("Frames", frameCount);
            telemetry.addData("File", "slot_" + pad(selectedSlot) + ".csv");
            telemetry.update();

        } catch (IOException e) {
            if (tmp.exists()) tmp.delete(); // don't leave a half-written temp behind
            telemetry.addLine("ERROR: Failed to save recording");
            telemetry.addData("Message", e.getMessage());
            telemetry.update();
        }
    }

    private static long parseTimestamp(String csvRow) {
        int comma = csvRow.indexOf(',');
        if (comma == -1) return 0;
        try {
            return Long.parseLong(csvRow.substring(0, comma));
        } catch (NumberFormatException e) {
            return 0;
        }
    }

    // ========================
    // UTILITY
    // ========================

    private static String formatDouble(double value, int decimals) {
        return String.format(Locale.US, "%." + decimals + "f", value);
    }

    private void sleep(long ms) {
        try {
            Thread.sleep(ms);
        } catch (InterruptedException e) {
            // Re-assert so the selection loop's isInterrupted() check can observe STOP
            // even on the legacy constructor path (no OpMode reference).
            Thread.currentThread().interrupt();
        }
    }

    public int getDeviceCount() {
        return motorNames.size() + servoNames.size() + crServoNames.size();
    }
}
