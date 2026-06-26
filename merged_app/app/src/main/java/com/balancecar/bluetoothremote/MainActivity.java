package com.balancecar.bluetoothremote;

import android.Manifest;
import android.annotation.SuppressLint;
import android.app.Activity;
import android.app.AlertDialog;
import android.bluetooth.BluetoothAdapter;
import android.bluetooth.BluetoothDevice;
import android.bluetooth.BluetoothSocket;
import android.content.pm.PackageManager;
import android.content.pm.ActivityInfo;
import android.content.SharedPreferences;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.graphics.Path;
import android.graphics.RadialGradient;
import android.graphics.RectF;
import android.graphics.Shader;
import android.graphics.Typeface;
import android.graphics.drawable.GradientDrawable;
import android.os.Build;
import android.os.Bundle;
import android.os.Handler;
import android.os.Looper;
import android.content.res.Configuration;
import android.view.Gravity;
import android.view.MotionEvent;
import android.view.View;
import android.view.Window;
import android.view.WindowManager;
import android.widget.ArrayAdapter;
import android.widget.Button;
import android.widget.EditText;
import android.widget.LinearLayout;
import android.widget.ScrollView;
import android.widget.Spinner;
import android.widget.TextView;
import android.widget.Toast;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.io.OutputStream;
import java.nio.charset.StandardCharsets;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import java.util.Set;
import java.util.UUID;

public class MainActivity extends Activity {
    private static final UUID SPP_UUID = UUID.fromString("00001101-0000-1000-8000-00805F9B34FB");
    private static final long CONTROL_PERIOD_MS = 50L;
    private static final float MAX_SPEED = 3.0f;
    private static final float MAX_TURN = 2.0f;
    private static final float DEAD_ZONE = 0.08f;
    private static final int COLOR_BG = Color.rgb(17, 19, 23);
    private static final int COLOR_PANEL = Color.rgb(30, 35, 41);
    private static final int COLOR_PANEL_ALT = Color.rgb(24, 28, 34);
    private static final int COLOR_STROKE = Color.rgb(61, 68, 78);
    private static final int COLOR_SCREEN = Color.rgb(9, 11, 15);
    private static final int COLOR_TEXT = Color.rgb(234, 238, 246);
    private static final int COLOR_MUTED = Color.rgb(145, 149, 160);
    private static final int COLOR_CYAN = Color.rgb(142, 113, 255);
    private static final int COLOR_PURPLE = Color.rgb(122, 91, 227);
    private static final int COLOR_GREEN = Color.rgb(72, 224, 93);
    private static final int COLOR_RED = Color.rgb(226, 72, 55);
    private static final int COLOR_BLUE = Color.rgb(77, 197, 255);
    private static final int COLOR_AMBER = Color.rgb(255, 184, 77);
    private static final int LOOP_SPEED = 0;
    private static final int LOOP_ANGLE = 1;
    private static final int LOOP_TURN = 2;
    private static final int PID_KP = 0;
    private static final int PID_KI = 1;
    private static final int PID_KD = 2;
    private static final int WAVE_POINTS = 140;
    private static final int SCREEN_CONTROL = 0;
    private static final int SCREEN_DEBUG = 1;
    private static final float MIN_WAVE_SCALE = 0.05f;
    private static final float MAX_WAVE_SCALE = 100.0f;
    private static final String[] LOOP_COMMANDS = {"SPD", "ANG", "TURN"};
    private static final String[] LOOP_LABELS = {"速度环", "角度环", "转向环"};
    private static final String[] PID_LABELS = {"Kp", "Ki", "Kd"};
    private static final String PREFS_NAME = "balance_car_app_state";
    private static final String PREF_SELECTED_LOOP = "selected_loop";
    private static final float[][] PID_MAX_DEFAULTS = {
        {3.0f, 2.0f, 1.0f},
        {30.0f, 2.0f, 20.0f},
        {10.0f, 2.0f, 5.0f}
    };
    private static final float[][] PID_DEFAULTS = {
        {0.8f, 0.3f, 0.0f},
        {11.0f, 0.25f, 8.0f},
        {0.0f, 0.0f, 0.0f}
    };

    private final Handler handler = new Handler(Looper.getMainLooper());
    private final List<BluetoothDevice> devices = new ArrayList<>();

    private BluetoothAdapter bluetoothAdapter;
    private BluetoothSocket socket;
    private OutputStream outputStream;
    private InputStream inputStream;
    private Thread receiveThread;
    private Spinner deviceSpinner;
    private TextView statusText;
    private TextView commandText;
    private LinearLayout bodyContainer;
    private Button screenSwitchButton;
    private Button connectButton;
    private TelemetryDisplayView telemetryView;
    private DebugWaveView debugWaveView;
    private final float[][] waveTargets = new float[3][WAVE_POINTS];
    private final float[][] waveActuals = new float[3][WAVE_POINTS];
    private final int[] waveSampleCounts = new int[3];
    private final float[] latestWaveTargets = new float[3];
    private final float[] latestWaveActuals = new float[3];
    private final float[] manualWaveScales = {0.25f, 5.0f, 0.25f};
    private final float[][] pidValues = new float[3][3];
    private final float[][] pidMaxValues = new float[3][3];
    private TextView[] pidValueTexts;
    private TextView speedValueText;
    private TextView turnValueText;
    private boolean running;
    private boolean debugMode;
    private boolean waveAutoScale = true;
    private volatile boolean bluetoothConnected;
    private volatile boolean bluetoothConnecting;
    private volatile int connectionToken;
    private volatile int disconnectedColor = COLOR_CYAN;
    private volatile String connectedDeviceName;
    private volatile String connectedDeviceAddress;
    private volatile String selectedDeviceAddress;
    private volatile String pendingDeviceName;
    private volatile String disconnectedMessage = "未连接";
    private int activeScreen = SCREEN_CONTROL;
    private int selectedLoop = LOOP_SPEED;
    private float currentSpeed;
    private float currentTurn;
    private float lastSentSpeed = Float.NaN;
    private float lastSentTurn = Float.NaN;

    private final Runnable pidSendTask = new Runnable() {
        @Override
        public void run() {
            sendSelectedPid();
        }
    };

    private final Runnable controlTask = new Runnable() {
        @Override
        public void run() {
            sendControlTargets(false);
            handler.postDelayed(this, CONTROL_PERIOD_MS);
        }
    };

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        requestWindowFeature(Window.FEATURE_NO_TITLE);
        getWindow().setFlags(WindowManager.LayoutParams.FLAG_FULLSCREEN, WindowManager.LayoutParams.FLAG_FULLSCREEN);
        getWindow().getDecorView().setSystemUiVisibility(
            View.SYSTEM_UI_FLAG_FULLSCREEN |
            View.SYSTEM_UI_FLAG_HIDE_NAVIGATION |
            View.SYSTEM_UI_FLAG_IMMERSIVE_STICKY |
            View.SYSTEM_UI_FLAG_LAYOUT_FULLSCREEN |
            View.SYSTEM_UI_FLAG_LAYOUT_HIDE_NAVIGATION |
            View.SYSTEM_UI_FLAG_LAYOUT_STABLE);
        setRequestedOrientation(ActivityInfo.SCREEN_ORIENTATION_LANDSCAPE);
        bluetoothAdapter = BluetoothAdapter.getDefaultAdapter();
        loadPidValues();
        setContentView(createUi());
        requestBluetoothPermission();
        loadPairedDevices();
        handler.postDelayed(controlTask, CONTROL_PERIOD_MS);
    }

    private View createUi() {
        LinearLayout root = new LinearLayout(this);
        root.setOrientation(LinearLayout.VERTICAL);
        root.setPadding(dp(8), dp(8), dp(8), dp(8));
        root.setBackgroundColor(COLOR_BG);

        LinearLayout topBar = row();
        topBar.setGravity(Gravity.CENTER_VERTICAL);
        boolean portrait = getResources().getConfiguration().orientation == Configuration.ORIENTATION_PORTRAIT;

        TextView title = new TextView(this);
        title.setText("Bluetooth");
        title.setTextSize(21);
        title.setTypeface(Typeface.DEFAULT_BOLD);
        title.setTextColor(COLOR_TEXT);

        statusText = new TextView(this);
        statusText.setTextSize(13);
        statusText.setTextColor(COLOR_CYAN);

        deviceSpinner = new Spinner(this);
        deviceSpinner.setBackground(panelBackground(COLOR_PANEL_ALT));
        screenSwitchButton = button("调试", COLOR_PANEL_ALT, v -> toggleScreen());
        connectButton = button("连接", COLOR_PURPLE, v -> connectSelectedDevice());

        if (portrait) {
            topBar.setOrientation(LinearLayout.VERTICAL);
            LinearLayout titleRow = row();
            titleRow.setGravity(Gravity.CENTER_VERTICAL);
            titleRow.addView(title, new LinearLayout.LayoutParams(dp(112), dp(42)));
            titleRow.addView(statusText, new LinearLayout.LayoutParams(0, dp(42), 1f));
            titleRow.addView(screenSwitchButton, compactButtonParams());
            topBar.addView(titleRow, new LinearLayout.LayoutParams(
                LinearLayout.LayoutParams.MATCH_PARENT,
                dp(46)));

            LinearLayout deviceRow = row();
            deviceRow.setGravity(Gravity.CENTER_VERTICAL);
            deviceRow.addView(deviceSpinner, new LinearLayout.LayoutParams(0, dp(44), 1f));
            deviceRow.addView(button("刷新", COLOR_PANEL_ALT, v -> loadPairedDevices()), compactButtonParams());
            deviceRow.addView(connectButton, compactButtonParams());
            topBar.addView(deviceRow, new LinearLayout.LayoutParams(
                LinearLayout.LayoutParams.MATCH_PARENT,
                dp(48)));
        } else {
            topBar.addView(title, new LinearLayout.LayoutParams(dp(120), LinearLayout.LayoutParams.WRAP_CONTENT));
            topBar.addView(statusText, new LinearLayout.LayoutParams(0, LinearLayout.LayoutParams.WRAP_CONTENT, 0.75f));
            topBar.addView(deviceSpinner, new LinearLayout.LayoutParams(0, dp(44), 1.7f));
            topBar.addView(button("刷新", COLOR_PANEL_ALT, v -> loadPairedDevices()), smallButtonParams());
            topBar.addView(connectButton, smallButtonParams());
            topBar.addView(screenSwitchButton, smallButtonParams());
        }
        root.addView(topBar);

        bodyContainer = new LinearLayout(this);
        bodyContainer.setOrientation(LinearLayout.VERTICAL);
        root.addView(bodyContainer, new LinearLayout.LayoutParams(
            LinearLayout.LayoutParams.MATCH_PARENT,
            0,
            1f));

        if (activeScreen == SCREEN_DEBUG) {
            showDebugScreen(false);
        } else {
            showControlScreen(false);
        }
        updateConnectionStatusUi();
        return root;
    }

    private void showControlScreen() {
        showControlScreen(true);
    }

    private void showControlScreen(boolean changeOrientation) {
        activeScreen = SCREEN_CONTROL;
        debugMode = false;
        if (changeOrientation && getResources().getConfiguration().orientation != Configuration.ORIENTATION_LANDSCAPE) {
            setRequestedOrientation(ActivityInfo.SCREEN_ORIENTATION_LANDSCAPE);
            return;
        }
        if (screenSwitchButton != null) {
            screenSwitchButton.setText("调试");
        }
        bodyContainer.removeAllViews();
        debugWaveView = null;

        LinearLayout main = row();
        main.setPadding(0, dp(8), 0, 0);

        LinearLayout leftPanel = panel("");
        TextView forwardLabel = padLabel("FORWARD");
        leftPanel.addView(forwardLabel);
        JoystickView speedStick = new JoystickView(this, "SPD");
        speedStick.setOnMoveListener((x, y, released) -> {
            float speed = applyDeadZone(-y) * MAX_SPEED;
            if (released) {
                speed = 0.0f;
            }
            setSpeedTarget(speed);
        });
        leftPanel.addView(speedStick, new LinearLayout.LayoutParams(LinearLayout.LayoutParams.MATCH_PARENT, 0, 1f));
        TextView backwardLabel = padLabel("BACKWARD");
        leftPanel.addView(backwardLabel);
        speedValueText = smallValueText(String.format(Locale.US, "SPD %.2f", currentSpeed));
        leftPanel.addView(speedValueText);
        main.addView(leftPanel, new LinearLayout.LayoutParams(0, LinearLayout.LayoutParams.MATCH_PARENT, 1.05f));

        LinearLayout centerPanel = panel("");
        View topGlow = glowBar();
        centerPanel.addView(topGlow);
        telemetryView = new TelemetryDisplayView(this);
        centerPanel.addView(telemetryView, new LinearLayout.LayoutParams(
            LinearLayout.LayoutParams.MATCH_PARENT,
            0,
            1f));

        commandText = smallValueText("等待命令");
        commandText.setTextSize(13);
        commandText.setTextColor(COLOR_MUTED);
        centerPanel.addView(commandText);

        LinearLayout actionRow = row();
        actionRow.setGravity(Gravity.CENTER);
        actionRow.setPadding(0, 0, 0, 0);
        Button startButton = button("START", Color.rgb(13, 25, 19), v -> {
            running = true;
            sendCommand("RUN 1", true);
        });
        startButton.setTextColor(COLOR_GREEN);
        Button stopButton = button("EMERGENCY\nSTOP", Color.rgb(62, 20, 16), v -> {
            setSpeedTarget(0.0f);
            setTurnTarget(0.0f);
            running = false;
            sendCommand("RUN 0", true);
        });
        stopButton.setTextColor(Color.rgb(255, 110, 91));
        stopButton.setTextSize(12);
        stopButton.setLineSpacing(0.0f, 0.88f);
        actionRow.addView(startButton, actionButtonParams());
        actionRow.addView(stopButton, actionButtonParams());
        centerPanel.addView(actionRow, new LinearLayout.LayoutParams(
            LinearLayout.LayoutParams.MATCH_PARENT,
            dp(54)));
        main.addView(centerPanel, new LinearLayout.LayoutParams(0, LinearLayout.LayoutParams.MATCH_PARENT, 1.2f));

        LinearLayout rightPanel = panel("");
        TextView turnLeftLabel = padLabel("TURN LEFT");
        rightPanel.addView(turnLeftLabel);
        JoystickView turnStick = new JoystickView(this, "TURN");
        turnStick.setOnMoveListener((x, y, released) -> {
            float turn = applyDeadZone(-x) * MAX_TURN;
            if (released) {
                turn = 0.0f;
            }
            setTurnTarget(turn);
        });
        rightPanel.addView(turnStick, new LinearLayout.LayoutParams(LinearLayout.LayoutParams.MATCH_PARENT, 0, 1f));
        TextView turnRightLabel = padLabel("TURN RIGHT");
        rightPanel.addView(turnRightLabel);
        turnValueText = smallValueText(String.format(Locale.US, "TURN %.2f", currentTurn));
        rightPanel.addView(turnValueText);
        main.addView(rightPanel, new LinearLayout.LayoutParams(0, LinearLayout.LayoutParams.MATCH_PARENT, 1.05f));

        bodyContainer.addView(main, new LinearLayout.LayoutParams(
            LinearLayout.LayoutParams.MATCH_PARENT,
            0,
            1f));
    }

    private void showDebugScreen() {
        showDebugScreen(true);
    }

    private void showDebugScreen(boolean changeOrientation) {
        activeScreen = SCREEN_DEBUG;
        debugMode = true;
        if (changeOrientation && getResources().getConfiguration().orientation != Configuration.ORIENTATION_PORTRAIT) {
            setRequestedOrientation(ActivityInfo.SCREEN_ORIENTATION_PORTRAIT);
            return;
        }
        if (screenSwitchButton != null) {
            screenSwitchButton.setText("控制");
        }
        bodyContainer.removeAllViews();
        telemetryView = null;

        LinearLayout debugRoot = new LinearLayout(this);
        debugRoot.setOrientation(LinearLayout.VERTICAL);
        debugRoot.setPadding(0, dp(8), 0, 0);

        LinearLayout wavePanel = panel("");
        LinearLayout waveHeader = row();
        waveHeader.setGravity(Gravity.CENTER_VERTICAL);
        TextView title = padLabel("PID 调试波形");
        waveHeader.addView(title, new LinearLayout.LayoutParams(0, LinearLayout.LayoutParams.WRAP_CONTENT, 0.95f));
        TextView legend = smallValueText("目标 " + LOOP_LABELS[selectedLoop] + "  /  实际");
        legend.setTextColor(COLOR_MUTED);
        legend.setTextSize(12);
        waveHeader.addView(legend, new LinearLayout.LayoutParams(0, LinearLayout.LayoutParams.WRAP_CONTENT, 1.0f));
        waveHeader.addView(button(waveAutoScale ? "自动" : "手动", waveAutoScale ? COLOR_PURPLE : COLOR_PANEL_ALT, v -> {
            waveAutoScale = !waveAutoScale;
            if (debugWaveView != null) {
                debugWaveView.invalidate();
            }
            showDebugScreen(false);
        }), waveButtonParams());
        waveHeader.addView(button("+", COLOR_PANEL_ALT, v -> zoomWave(0.75f)), waveIconButtonParams());
        waveHeader.addView(button("-", COLOR_PANEL_ALT, v -> zoomWave(1.25f)), waveIconButtonParams());
        wavePanel.addView(waveHeader);

        debugWaveView = new DebugWaveView(this);
        debugWaveView.setLoop(selectedLoop);
        wavePanel.addView(debugWaveView, new LinearLayout.LayoutParams(
            LinearLayout.LayoutParams.MATCH_PARENT,
            0,
            1f));
        debugRoot.addView(wavePanel, new LinearLayout.LayoutParams(
            LinearLayout.LayoutParams.MATCH_PARENT,
            0,
            0.85f));

        LinearLayout pidPanel = panel("");
        pidPanel.addView(createLoopTabs());
        ScrollView pidScroll = new ScrollView(this);
        pidScroll.setFillViewport(false);
        LinearLayout pidContent = new LinearLayout(this);
        pidContent.setOrientation(LinearLayout.VERTICAL);
        pidContent.addView(createPidSliders());

        LinearLayout actionRow = row();
        actionRow.setGravity(Gravity.CENTER);
        actionRow.addView(button("发送当前PID", COLOR_PURPLE, v -> sendSelectedPid()), tallActionButtonParams());
        actionRow.addView(button("清PID历史", COLOR_PANEL_ALT, v -> sendCommand("PIDRST", true)), tallActionButtonParams());
        actionRow.addView(button("上限", COLOR_PANEL_ALT, v -> showPidLimitDialog()), tallActionButtonParams());
        pidContent.addView(actionRow, new LinearLayout.LayoutParams(
            LinearLayout.LayoutParams.MATCH_PARENT,
            dp(62)));

        commandText = smallValueText("等待调试命令");
        commandText.setTextSize(13);
        commandText.setTextColor(COLOR_MUTED);
        pidContent.addView(commandText, new LinearLayout.LayoutParams(
            LinearLayout.LayoutParams.MATCH_PARENT,
            dp(44)));
        pidScroll.addView(pidContent, new ScrollView.LayoutParams(
            ScrollView.LayoutParams.MATCH_PARENT,
            ScrollView.LayoutParams.WRAP_CONTENT));
        pidPanel.addView(pidScroll, new LinearLayout.LayoutParams(
            LinearLayout.LayoutParams.MATCH_PARENT,
            0,
            1f));

        debugRoot.addView(pidPanel, new LinearLayout.LayoutParams(
            LinearLayout.LayoutParams.MATCH_PARENT,
            0,
            1.35f));

        bodyContainer.addView(debugRoot, new LinearLayout.LayoutParams(
            LinearLayout.LayoutParams.MATCH_PARENT,
            LinearLayout.LayoutParams.MATCH_PARENT));
    }

    private void toggleScreen() {
        if (debugMode) {
            showControlScreen();
        } else {
            showDebugScreen();
        }
    }

    private void zoomWave(float factor) {
        waveAutoScale = false;
        manualWaveScales[selectedLoop] = clampWaveScale(manualWaveScales[selectedLoop] * factor);
        if (debugWaveView != null) {
            debugWaveView.invalidate();
        }
        showDebugScreen(false);
    }

    private float clampWaveScale(float scale) {
        if (scale < MIN_WAVE_SCALE) {
            return MIN_WAVE_SCALE;
        }
        if (scale > MAX_WAVE_SCALE) {
            return MAX_WAVE_SCALE;
        }
        return scale;
    }

    @Override
    public void onConfigurationChanged(Configuration newConfig) {
        super.onConfigurationChanged(newConfig);
        setContentView(createUi());
        loadPairedDevices();
    }

    private LinearLayout createLoopTabs() {
        LinearLayout tabs = row();
        tabs.setGravity(Gravity.CENTER);
        for (int i = 0; i < LOOP_LABELS.length; i++) {
            final int loop = i;
            int color = selectedLoop == loop ? COLOR_PURPLE : COLOR_PANEL_ALT;
            Button tab = button(LOOP_LABELS[i], color, v -> {
                selectedLoop = loop;
                savePidValues();
                showDebugScreen(false);
            });
            tabs.addView(tab, tallActionButtonParams());
        }
        return tabs;
    }

    private LinearLayout createPidSliders() {
        LinearLayout sliders = new LinearLayout(this);
        sliders.setOrientation(LinearLayout.VERTICAL);
        sliders.setPadding(0, dp(6), 0, 0);
        pidValueTexts = new TextView[3];
        for (int i = 0; i < PID_LABELS.length; i++) {
            sliders.addView(createPidSlider(i), new LinearLayout.LayoutParams(
                LinearLayout.LayoutParams.MATCH_PARENT,
                dp(64)));
        }
        return sliders;
    }

    private View createPidSlider(int pidIndex) {
        LinearLayout row = new LinearLayout(this);
        row.setOrientation(LinearLayout.HORIZONTAL);
        row.setGravity(Gravity.CENTER_VERTICAL);
        row.setPadding(0, dp(4), 0, dp(4));

        TextView name = padLabel(PID_LABELS[pidIndex]);
        name.setTextSize(18);
        row.addView(name, new LinearLayout.LayoutParams(dp(56), LinearLayout.LayoutParams.WRAP_CONTENT));

        PidSliderView slider = new PidSliderView(this);
        final int loop = selectedLoop;
        slider.setProgressFraction(valueToProgressFraction(loop, pidIndex, pidValues[loop][pidIndex]));
        slider.setOnValueChangeListener((fraction, released) -> {
            pidValues[loop][pidIndex] = fractionToValue(loop, pidIndex, fraction);
            savePidValues();
            updatePidValueText(pidIndex);
            if (released) {
                sendSelectedPid();
            } else {
                schedulePidSend();
            }
        });
        row.addView(slider, new LinearLayout.LayoutParams(0, dp(50), 1f));

        TextView value = smallValueText("");
        value.setGravity(Gravity.RIGHT | Gravity.CENTER_VERTICAL);
        value.setTextSize(17);
        pidValueTexts[pidIndex] = value;
        updatePidValueText(pidIndex);
        row.addView(value, new LinearLayout.LayoutParams(dp(86), LinearLayout.LayoutParams.WRAP_CONTENT));
        return row;
    }

    private void resetPidValues() {
        for (int loop = 0; loop < PID_DEFAULTS.length; loop++) {
            System.arraycopy(PID_DEFAULTS[loop], 0, pidValues[loop], 0, PID_DEFAULTS[loop].length);
            System.arraycopy(PID_MAX_DEFAULTS[loop], 0, pidMaxValues[loop], 0, PID_MAX_DEFAULTS[loop].length);
        }
    }

    private void loadPidValues() {
        resetPidValues();
        SharedPreferences prefs = getSharedPreferences(PREFS_NAME, MODE_PRIVATE);
        selectedLoop = prefs.getInt(PREF_SELECTED_LOOP, LOOP_SPEED);
        if (selectedLoop < LOOP_SPEED || selectedLoop > LOOP_TURN) {
            selectedLoop = LOOP_SPEED;
        }
        for (int loop = 0; loop < PID_DEFAULTS.length; loop++) {
            for (int pid = 0; pid < PID_LABELS.length; pid++) {
                float max = prefs.getFloat(pidMaxKey(loop, pid), pidMaxValues[loop][pid]);
                float value = prefs.getFloat(pidValueKey(loop, pid), pidValues[loop][pid]);
                if (max <= 0.0f) {
                    max = PID_MAX_DEFAULTS[loop][pid];
                }
                if (value < 0.0f) {
                    value = 0.0f;
                } else if (value > max) {
                    value = max;
                }
                pidMaxValues[loop][pid] = max;
                pidValues[loop][pid] = value;
            }
        }
    }

    private void savePidValues() {
        SharedPreferences.Editor editor = getSharedPreferences(PREFS_NAME, MODE_PRIVATE).edit();
        editor.putInt(PREF_SELECTED_LOOP, selectedLoop);
        for (int loop = 0; loop < PID_DEFAULTS.length; loop++) {
            for (int pid = 0; pid < PID_LABELS.length; pid++) {
                editor.putFloat(pidValueKey(loop, pid), pidValues[loop][pid]);
                editor.putFloat(pidMaxKey(loop, pid), pidMaxValues[loop][pid]);
            }
        }
        editor.apply();
    }

    private String pidValueKey(int loop, int pid) {
        return "pid_value_" + loop + "_" + pid;
    }

    private String pidMaxKey(int loop, int pid) {
        return "pid_max_" + loop + "_" + pid;
    }

    private int valueToProgress(int loop, int pidIndex, float value) {
        float max = pidMaxValues[loop][pidIndex];
        if (max <= 0.0f) {
            return 0;
        }
        int progress = Math.round(value / max * 1000.0f);
        if (progress < 0) {
            return 0;
        }
        if (progress > 1000) {
            return 1000;
        }
        return progress;
    }

    private float valueToProgressFraction(int loop, int pidIndex, float value) {
        float max = pidMaxValues[loop][pidIndex];
        if (max <= 0.0f) {
            return 0.0f;
        }
        float fraction = value / max;
        if (fraction < 0.0f) {
            return 0.0f;
        }
        if (fraction > 1.0f) {
            return 1.0f;
        }
        return fraction;
    }

    private float progressToValue(int loop, int pidIndex, int progress) {
        return pidMaxValues[loop][pidIndex] * progress / 1000.0f;
    }

    private float fractionToValue(int loop, int pidIndex, float fraction) {
        if (fraction < 0.0f) {
            fraction = 0.0f;
        } else if (fraction > 1.0f) {
            fraction = 1.0f;
        }
        return pidMaxValues[loop][pidIndex] * fraction;
    }

    private void showPidLimitDialog() {
        LinearLayout content = new LinearLayout(this);
        content.setOrientation(LinearLayout.VERTICAL);
        content.setPadding(dp(18), dp(10), dp(18), 0);

        TextView hint = smallValueText(LOOP_LABELS[selectedLoop] + " 滑杆上限");
        hint.setTextColor(COLOR_MUTED);
        content.addView(hint, new LinearLayout.LayoutParams(
            LinearLayout.LayoutParams.MATCH_PARENT,
            LinearLayout.LayoutParams.WRAP_CONTENT));

        EditText[] inputs = new EditText[3];
        for (int i = 0; i < PID_LABELS.length; i++) {
            inputs[i] = new EditText(this);
            inputs[i].setText(String.format(Locale.US, "%.3f", pidMaxValues[selectedLoop][i]));
            inputs[i].setSingleLine(true);
            inputs[i].setSelectAllOnFocus(true);
            inputs[i].setTextColor(COLOR_TEXT);
            inputs[i].setHintTextColor(COLOR_MUTED);
            inputs[i].setTextSize(18);
            inputs[i].setHint(PID_LABELS[i] + " Max");
            inputs[i].setInputType(android.text.InputType.TYPE_CLASS_NUMBER |
                android.text.InputType.TYPE_NUMBER_FLAG_DECIMAL);
            content.addView(inputs[i], new LinearLayout.LayoutParams(
                LinearLayout.LayoutParams.MATCH_PARENT,
                dp(58)));
        }

        AlertDialog dialog = new AlertDialog.Builder(this)
            .setTitle("设置 PID 上限")
            .setView(content)
            .setNegativeButton("取消", null)
            .setPositiveButton("保存", null)
            .create();
        dialog.setOnShowListener(d -> dialog.getButton(AlertDialog.BUTTON_POSITIVE).setOnClickListener(v -> {
            float[] next = new float[3];
            for (int i = 0; i < inputs.length; i++) {
                try {
                    next[i] = Float.parseFloat(inputs[i].getText().toString().trim());
                } catch (NumberFormatException e) {
                    toast("请输入有效的 " + PID_LABELS[i] + " 上限");
                    return;
                }
                if (next[i] <= 0.0f) {
                    toast(PID_LABELS[i] + " 上限必须大于 0");
                    return;
                }
            }
            applyPidLimits(next);
            dialog.dismiss();
        }));
        dialog.show();
    }

    private void applyPidLimits(float[] next) {
        for (int i = 0; i < PID_LABELS.length; i++) {
            pidMaxValues[selectedLoop][i] = next[i];
            if (pidValues[selectedLoop][i] > next[i]) {
                pidValues[selectedLoop][i] = next[i];
            }
        }
        savePidValues();
        showDebugScreen(false);
    }

    private void updatePidValueText(int pidIndex) {
        if (pidValueTexts == null || pidValueTexts[pidIndex] == null) {
            return;
        }
        float value = pidValues[selectedLoop][pidIndex];
        float max = pidMaxValues[selectedLoop][pidIndex];
        pidValueTexts[pidIndex].setText(String.format(Locale.US, "%.3f\n/%.3f", value, max));
    }

    private void schedulePidSend() {
        handler.removeCallbacks(pidSendTask);
        handler.postDelayed(pidSendTask, 140L);
    }

    private void sendSelectedPid() {
        handler.removeCallbacks(pidSendTask);
        sendCommand(String.format(Locale.US,
            "PID %s %.3f %.3f %.3f",
            LOOP_COMMANDS[selectedLoop],
            pidValues[selectedLoop][PID_KP],
            pidValues[selectedLoop][PID_KI],
            pidValues[selectedLoop][PID_KD]), true);
    }

    private void setSpeedTarget(float speed) {
        currentSpeed = speed;
        speedValueText.setText(String.format(Locale.US, "SPD %.2f", currentSpeed));
        sendControlTargets(true);
    }

    private void setTurnTarget(float turn) {
        currentTurn = turn;
        turnValueText.setText(String.format(Locale.US, "TURN %.2f", currentTurn));
        sendControlTargets(true);
    }

    private float applyDeadZone(float value) {
        if (Math.abs(value) < DEAD_ZONE) {
            return 0.0f;
        }
        return value;
    }

    private void sendControlTargets(boolean forceChanged) {
        if (!running) {
            return;
        }
        sendCommand(String.format(Locale.US, "CTL %.2f %.2f", currentSpeed, currentTurn), false);
        lastSentSpeed = currentSpeed;
        lastSentTurn = currentTurn;
    }

    private LinearLayout panel(String titleText) {
        LinearLayout panel = new LinearLayout(this);
        panel.setOrientation(LinearLayout.VERTICAL);
        panel.setPadding(dp(14), dp(8), dp(14), dp(8));
        LinearLayout.LayoutParams params = new LinearLayout.LayoutParams(0, LinearLayout.LayoutParams.MATCH_PARENT, 1f);
        params.setMargins(dp(5), 0, dp(5), 0);
        panel.setLayoutParams(params);
        panel.setBackground(panelBackground(COLOR_PANEL));

        if (!titleText.isEmpty()) {
            TextView title = padLabel(titleText);
            panel.addView(title);
        }
        return panel;
    }

    private TextView padLabel(String text) {
        TextView view = new TextView(this);
        view.setText(text);
        view.setTextSize(13);
        view.setTextColor(COLOR_TEXT);
        view.setGravity(Gravity.CENTER);
        view.setTypeface(Typeface.DEFAULT_BOLD);
        view.setPadding(0, dp(4), 0, dp(4));
        return view;
    }

    private TextView smallValueText(String text) {
        TextView view = valueText(text);
        view.setTextSize(14);
        view.setTextColor(COLOR_TEXT);
        return view;
    }

    private TextView valueText(String text) {
        TextView view = new TextView(this);
        view.setText(text);
        view.setTextSize(16);
        view.setTextColor(COLOR_TEXT);
        view.setGravity(Gravity.CENTER);
        view.setPadding(0, dp(7), 0, dp(7));
        return view;
    }

    private View glowBar() {
        View view = new View(this);
        GradientDrawable drawable = new GradientDrawable(
            GradientDrawable.Orientation.LEFT_RIGHT,
            new int[] {
                Color.TRANSPARENT,
                COLOR_PURPLE,
                Color.rgb(185, 151, 255),
                COLOR_PURPLE,
                Color.TRANSPARENT
            });
        drawable.setCornerRadius(dp(4));
        view.setBackground(drawable);
        LinearLayout.LayoutParams params = new LinearLayout.LayoutParams(dp(86), dp(5));
        params.gravity = Gravity.CENTER_HORIZONTAL;
        params.setMargins(0, 0, 0, dp(8));
        view.setLayoutParams(params);
        return view;
    }

    private GradientDrawable panelBackground(int color) {
        GradientDrawable drawable = new GradientDrawable(
            GradientDrawable.Orientation.TOP_BOTTOM,
            new int[] { Color.rgb(39, 44, 51), color, Color.rgb(19, 22, 27) });
        drawable.setStroke(dp(1), COLOR_STROKE);
        drawable.setCornerRadius(dp(18));
        return drawable;
    }

    private GradientDrawable buttonBackground(int color) {
        GradientDrawable drawable = new GradientDrawable();
        drawable.setColor(color);
        drawable.setStroke(dp(2), brighten(color));
        drawable.setCornerRadius(dp(6));
        return drawable;
    }

    private int brighten(int color) {
        int r = Math.min(255, (int)(Color.red(color) * 1.25f + 12));
        int g = Math.min(255, (int)(Color.green(color) * 1.25f + 12));
        int b = Math.min(255, (int)(Color.blue(color) * 1.25f + 12));
        return Color.rgb(r, g, b);
    }

    private Button button(String text, int color, View.OnClickListener listener) {
        Button button = new Button(this);
        button.setText(text);
        button.setTextSize(14);
        button.setTextColor(COLOR_TEXT);
        button.setTypeface(Typeface.DEFAULT_BOLD);
        button.setAllCaps(false);
        button.setGravity(Gravity.CENTER);
        button.setIncludeFontPadding(false);
        button.setPadding(0, 0, 0, 0);
        button.setBackground(buttonBackground(color));
        button.setOnClickListener(listener);
        return button;
    }

    private LinearLayout row() {
        LinearLayout row = new LinearLayout(this);
        row.setOrientation(LinearLayout.HORIZONTAL);
        return row;
    }

    private LinearLayout.LayoutParams smallButtonParams() {
        LinearLayout.LayoutParams params = new LinearLayout.LayoutParams(dp(74), dp(44));
        params.setMargins(dp(4), 0, 0, 0);
        return params;
    }

    private LinearLayout.LayoutParams compactButtonParams() {
        LinearLayout.LayoutParams params = new LinearLayout.LayoutParams(dp(72), dp(42));
        params.setMargins(dp(5), 0, 0, 0);
        return params;
    }

    private LinearLayout.LayoutParams waveButtonParams() {
        LinearLayout.LayoutParams params = new LinearLayout.LayoutParams(dp(54), dp(34));
        params.setMargins(dp(4), 0, 0, 0);
        return params;
    }

    private LinearLayout.LayoutParams waveIconButtonParams() {
        LinearLayout.LayoutParams params = new LinearLayout.LayoutParams(dp(38), dp(34));
        params.setMargins(dp(4), 0, 0, 0);
        return params;
    }

    private LinearLayout.LayoutParams actionButtonParams() {
        LinearLayout.LayoutParams params = new LinearLayout.LayoutParams(0, dp(48), 1f);
        params.gravity = Gravity.CENTER_VERTICAL;
        params.setMargins(dp(5), 0, dp(5), 0);
        return params;
    }

    private LinearLayout.LayoutParams tallActionButtonParams() {
        LinearLayout.LayoutParams params = new LinearLayout.LayoutParams(0, dp(56), 1f);
        params.gravity = Gravity.CENTER_VERTICAL;
        params.setMargins(dp(5), dp(4), dp(5), dp(4));
        return params;
    }

    private int dp(int value) {
        return (int) (value * getResources().getDisplayMetrics().density + 0.5f);
    }

    private void requestBluetoothPermission() {
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.S &&
            checkSelfPermission(Manifest.permission.BLUETOOTH_CONNECT) != PackageManager.PERMISSION_GRANTED) {
            requestPermissions(new String[] {
                Manifest.permission.BLUETOOTH_CONNECT,
                Manifest.permission.BLUETOOTH_SCAN
            }, 100);
        }
    }

    @SuppressLint("MissingPermission")
    private void loadPairedDevices() {
        if (bluetoothAdapter == null) {
            toast("此手机不支持蓝牙");
            return;
        }
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.S &&
            checkSelfPermission(Manifest.permission.BLUETOOTH_CONNECT) != PackageManager.PERMISSION_GRANTED) {
            requestBluetoothPermission();
            return;
        }

        devices.clear();
        List<String> names = new ArrayList<>();
        Set<BluetoothDevice> bondedDevices = bluetoothAdapter.getBondedDevices();
        for (BluetoothDevice device : bondedDevices) {
            devices.add(device);
            String name = device.getName();
            names.add((name == null ? "Unknown" : name) + "  " + device.getAddress());
        }
        if (names.isEmpty()) {
            names.add("未找到已配对设备");
        }
        deviceSpinner.setAdapter(new ArrayAdapter<>(this, android.R.layout.simple_spinner_dropdown_item, names));
        restoreSelectedDevice();
    }

    @SuppressLint("MissingPermission")
    private void connectSelectedDevice() {
        if (devices.isEmpty()) {
            toast("请先在系统蓝牙中配对 HC-05/HC-06");
            return;
        }
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.S &&
            checkSelfPermission(Manifest.permission.BLUETOOTH_CONNECT) != PackageManager.PERMISSION_GRANTED) {
            requestBluetoothPermission();
            return;
        }

        BluetoothDevice device = devices.get(deviceSpinner.getSelectedItemPosition());
        selectedDeviceAddress = device.getAddress();
        setConnectingState(deviceName(device));
        new Thread(() -> {
            int token = connectionToken;
            try {
                closeSocket();
                BluetoothSocket nextSocket = device.createRfcommSocketToServiceRecord(SPP_UUID);
                bluetoothAdapter.cancelDiscovery();
                nextSocket.connect();
                synchronized (MainActivity.this) {
                    socket = nextSocket;
                    outputStream = socket.getOutputStream();
                    inputStream = socket.getInputStream();
                }
                setConnectedState(deviceName(device), device.getAddress(), token);
                startReceiveThread(token);
            } catch (IOException e) {
                closeSocket();
                setDisconnectedState("连接失败", COLOR_RED, token);
                runOnUiThread(() -> toast("连接失败，请确认模块已配对且未被其他APP占用"));
            }
        }).start();
    }

    private void startReceiveThread(int token) {
        receiveThread = new Thread(() -> {
            InputStream stream = inputStream;
            try {
                BufferedReader reader = new BufferedReader(new InputStreamReader(stream, StandardCharsets.UTF_8));
                String line;
                while ((line = reader.readLine()) != null) {
                    String received = line.trim();
                    runOnUiThread(() -> handleTelemetry(received));
                }
            } catch (IOException ignored) {
            } finally {
                setDisconnectedState("连接已断开", COLOR_RED, token);
                closeSocketForToken(token);
            }
        });
        receiveThread.setName("BalanceCarBtRx");
        receiveThread.start();
    }

    private void restoreSelectedDevice() {
        if (deviceSpinner == null || selectedDeviceAddress == null) {
            return;
        }
        for (int i = 0; i < devices.size(); i++) {
            if (selectedDeviceAddress.equals(devices.get(i).getAddress())) {
                deviceSpinner.setSelection(i);
                return;
            }
        }
    }

    private String deviceName(BluetoothDevice device) {
        String name = device.getName();
        return name == null || name.trim().isEmpty() ? device.getAddress() : name;
    }

    private synchronized boolean hasActiveConnection() {
        return bluetoothConnected && outputStream != null && socket != null && socket.isConnected();
    }

    private synchronized void setConnectingState(String deviceName) {
        connectionToken++;
        bluetoothConnecting = true;
        bluetoothConnected = false;
        pendingDeviceName = deviceName;
        disconnectedMessage = "未连接";
        disconnectedColor = COLOR_CYAN;
        updateConnectionStatusUi();
    }

    private synchronized void setConnectedState(String deviceName, String deviceAddress, int token) {
        if (token != connectionToken) {
            return;
        }
        bluetoothConnecting = false;
        bluetoothConnected = true;
        connectedDeviceName = deviceName;
        connectedDeviceAddress = deviceAddress;
        selectedDeviceAddress = deviceAddress;
        pendingDeviceName = null;
        runOnUiThread(this::updateConnectionStatusUi);
    }

    private synchronized void setDisconnectedState(String message, int color, int token) {
        if (token != connectionToken) {
            return;
        }
        bluetoothConnecting = false;
        bluetoothConnected = false;
        connectedDeviceName = null;
        connectedDeviceAddress = null;
        pendingDeviceName = null;
        disconnectedMessage = message;
        disconnectedColor = color;
        runOnUiThread(this::updateConnectionStatusUi);
    }

    private void updateConnectionStatusUi() {
        if (Looper.myLooper() != Looper.getMainLooper()) {
            runOnUiThread(this::updateConnectionStatusUi);
            return;
        }
        if (statusText == null) {
            return;
        }
        if (bluetoothConnected) {
            statusText.setText("已连接 " + connectedDeviceName);
            statusText.setTextColor(COLOR_GREEN);
            if (connectButton != null) {
                connectButton.setText("重连");
            }
            return;
        }
        if (bluetoothConnecting) {
            statusText.setText("正在连接 " + pendingDeviceName);
            statusText.setTextColor(COLOR_AMBER);
            if (connectButton != null) {
                connectButton.setText("连接中");
            }
            return;
        }
        statusText.setText(disconnectedMessage);
        statusText.setTextColor(disconnectedColor);
        if (connectButton != null) {
            connectButton.setText("连接");
        }
    }

    private void handleTelemetry(String line) {
        String cleanLine = line.trim();
        if (cleanLine.startsWith("DBG")) {
            handleDebugTelemetry(cleanLine.length() > 3 ? cleanLine.substring(3).trim() : "");
            return;
        }
        if (!cleanLine.startsWith("OLED ")) {
            return;
        }
        String payload = cleanLine.substring(5);
        String[] parts = payload.split("\\|");
        if (telemetryView != null) {
            telemetryView.setTelemetry(
                parts.length > 0 ? parts[0] : "Temp: --.- C",
                parts.length > 1 ? parts[1] : "Humi: -- %",
                parts.length > 2 ? parts[2] : "Weight: -- g",
                parts.length > 3 ? parts[3] : "ADC: ----");
        }
    }

    private void handleDebugTelemetry(String payload) {
        String[] parts = payload.trim().split("[\\s,;:=]+");
        if (parts.length < 3) {
            return;
        }
        int loop = loopFromCommand(parts[0].toUpperCase(Locale.US));
        if (loop < 0) {
            return;
        }
        try {
            float target = Float.parseFloat(parts[1]);
            float actual = Float.parseFloat(parts[2]);
            pushWaveSample(loop, target, actual);
        } catch (NumberFormatException ignored) {
        }
    }

    private int loopFromCommand(String command) {
        for (int i = 0; i < LOOP_COMMANDS.length; i++) {
            if (LOOP_COMMANDS[i].equals(command)) {
                return i;
            }
        }
        return -1;
    }

    private void pushWaveSample(int loop, float target, float actual) {
        System.arraycopy(waveTargets[loop], 1, waveTargets[loop], 0, WAVE_POINTS - 1);
        System.arraycopy(waveActuals[loop], 1, waveActuals[loop], 0, WAVE_POINTS - 1);
        waveTargets[loop][WAVE_POINTS - 1] = target;
        waveActuals[loop][WAVE_POINTS - 1] = actual;
        latestWaveTargets[loop] = target;
        latestWaveActuals[loop] = actual;
        if (waveSampleCounts[loop] < WAVE_POINTS) {
            waveSampleCounts[loop]++;
        }
        if (debugWaveView != null) {
            debugWaveView.invalidate();
        }
    }

    private synchronized void sendCommand(String command, boolean showToastWhenDisconnected) {
        if (commandText != null) {
            commandText.setText("发送: " + command);
        }
        OutputStream out = outputStream;
        if (out == null || !hasActiveConnection()) {
            if (showToastWhenDisconnected) {
                toast("蓝牙未连接");
            }
            updateConnectionStatusUi();
            return;
        }
        try {
            out.write((command + "\n").getBytes(StandardCharsets.UTF_8));
            out.flush();
        } catch (IOException e) {
            int token = connectionToken;
            setDisconnectedState("发送失败，连接已断开", COLOR_RED, token);
            closeSocketForToken(token);
        }
    }

    private synchronized void closeSocketForToken(int token) {
        if (token != connectionToken) {
            return;
        }
        closeSocket();
    }

    private synchronized void closeSocket() {
        try {
            if (outputStream != null) {
                outputStream.close();
            }
        } catch (IOException ignored) {
        }
        try {
            if (inputStream != null) {
                inputStream.close();
            }
        } catch (IOException ignored) {
        }
        try {
            if (socket != null) {
                socket.close();
            }
        } catch (IOException ignored) {
        }
        outputStream = null;
        inputStream = null;
        socket = null;
    }

    private void toast(String text) {
        Toast.makeText(this, text, Toast.LENGTH_SHORT).show();
    }

    @Override
    protected void onDestroy() {
        handler.removeCallbacks(controlTask);
        handler.removeCallbacks(pidSendTask);
        closeSocket();
        super.onDestroy();
    }

    private final class PidSliderView extends View {
        private final Paint paint = new Paint(Paint.ANTI_ALIAS_FLAG);
        private OnSliderValueChangeListener listener;
        private float fraction;

        PidSliderView(Activity activity) {
            super(activity);
            setMinimumHeight(dp(44));
        }

        void setProgressFraction(float fraction) {
            this.fraction = clampFraction(fraction);
            invalidate();
        }

        void setOnValueChangeListener(OnSliderValueChangeListener listener) {
            this.listener = listener;
        }

        @Override
        protected void onDraw(Canvas canvas) {
            super.onDraw(canvas);
            float thumbRadius = dp(17);
            float trackHeight = dp(12);
            float left = thumbRadius + dp(2);
            float right = getWidth() - thumbRadius - dp(2);
            float centerY = getHeight() * 0.5f;
            float thumbX = left + (right - left) * fraction;

            paint.setStyle(Paint.Style.FILL);
            paint.setColor(Color.rgb(66, 72, 83));
            canvas.drawRoundRect(new RectF(left, centerY - trackHeight * 0.5f, right, centerY + trackHeight * 0.5f),
                trackHeight * 0.5f, trackHeight * 0.5f, paint);

            paint.setColor(COLOR_PURPLE);
            canvas.drawRoundRect(new RectF(left, centerY - trackHeight * 0.5f, thumbX, centerY + trackHeight * 0.5f),
                trackHeight * 0.5f, trackHeight * 0.5f, paint);

            paint.setColor(Color.rgb(95, 68, 190));
            canvas.drawCircle(thumbX, centerY, thumbRadius + dp(3), paint);
            paint.setColor(Color.rgb(184, 159, 255));
            canvas.drawCircle(thumbX, centerY, thumbRadius, paint);
        }

        @Override
        public boolean onTouchEvent(MotionEvent event) {
            if (event.getAction() == MotionEvent.ACTION_DOWN) {
                getParent().requestDisallowInterceptTouchEvent(true);
                updateFromTouch(event.getX(), false);
                return true;
            }
            if (event.getAction() == MotionEvent.ACTION_MOVE) {
                updateFromTouch(event.getX(), false);
                return true;
            }
            if (event.getAction() == MotionEvent.ACTION_UP || event.getAction() == MotionEvent.ACTION_CANCEL) {
                updateFromTouch(event.getX(), true);
                getParent().requestDisallowInterceptTouchEvent(false);
                return true;
            }
            return true;
        }

        private void updateFromTouch(float x, boolean released) {
            float thumbRadius = dp(17);
            float left = thumbRadius + dp(2);
            float right = getWidth() - thumbRadius - dp(2);
            if (right <= left) {
                return;
            }
            fraction = clampFraction((x - left) / (right - left));
            invalidate();
            if (listener != null) {
                listener.onValueChanged(fraction, released);
            }
        }

        private float clampFraction(float value) {
            if (value < 0.0f) {
                return 0.0f;
            }
            if (value > 1.0f) {
                return 1.0f;
            }
            return value;
        }
    }

    private interface OnSliderValueChangeListener {
        void onValueChanged(float fraction, boolean released);
    }

    private final class DebugWaveView extends View {
        private final Paint paint = new Paint(Paint.ANTI_ALIAS_FLAG);
        private int loop;

        DebugWaveView(Activity activity) {
            super(activity);
            setMinimumHeight(activity.getResources().getDisplayMetrics().densityDpi / 2);
        }

        void setLoop(int loop) {
            this.loop = loop;
            invalidate();
        }

        @Override
        protected void onDraw(Canvas canvas) {
            super.onDraw(canvas);
            float w = getWidth();
            float h = getHeight();
            if (w <= 0.0f || h <= 0.0f) {
                return;
            }
            float left = dp(34);
            float top = dp(10);
            float right = w - dp(12);
            float bottom = h - dp(24);
            if (right <= left + dp(20) || bottom <= top + dp(20)) {
                return;
            }
            RectF plot = new RectF(left, top, right, bottom);

            paint.setStyle(Paint.Style.FILL);
            paint.setColor(COLOR_SCREEN);
            canvas.drawRoundRect(plot, dp(8), dp(8), paint);

            drawGrid(canvas, plot);

            float scale = waveScale(loop);
            drawWave(canvas, plot, waveTargets[loop], scale, COLOR_AMBER, 3.0f);
            drawWave(canvas, plot, waveActuals[loop], scale, COLOR_BLUE, 3.0f);
            drawLatestMarkers(canvas, plot, scale);

            paint.setStyle(Paint.Style.FILL);
            paint.setTypeface(Typeface.DEFAULT_BOLD);
            paint.setTextSize(22.0f);
            paint.setTextAlign(Paint.Align.LEFT);
            paint.setColor(COLOR_AMBER);
            canvas.drawText("Target", plot.left + dp(10), plot.top + dp(26), paint);
            paint.setColor(COLOR_BLUE);
            canvas.drawText("Actual", plot.left + dp(104), plot.top + dp(26), paint);

            paint.setTextAlign(Paint.Align.LEFT);
            paint.setTextSize(16.0f);
            paint.setTypeface(Typeface.DEFAULT);
            paint.setColor(COLOR_MUTED);
            canvas.drawText(String.format(Locale.US,
                "T %.3f  A %.3f",
                latestWaveTargets[loop],
                latestWaveActuals[loop]), plot.left + dp(10), plot.bottom - dp(8), paint);

            paint.setTextAlign(Paint.Align.RIGHT);
            paint.setTextSize(18.0f);
            paint.setColor(COLOR_MUTED);
            canvas.drawText(String.format(Locale.US, "+%.2f", scale), plot.left - dp(4), plot.top + dp(6), paint);
            canvas.drawText("0", plot.left - dp(4), plot.centerY() + dp(6), paint);
            canvas.drawText(String.format(Locale.US, "-%.2f", scale), plot.left - dp(4), plot.bottom, paint);

            paint.setTextAlign(Paint.Align.RIGHT);
            paint.setTextSize(17.0f);
            paint.setColor(waveSampleCounts[loop] > 0 ? (waveAutoScale ? COLOR_PURPLE : COLOR_MUTED) : COLOR_AMBER);
            canvas.drawText(waveSampleCounts[loop] > 0 ? (waveAutoScale ? "AUTO" : "MANUAL") : "NO DBG",
                plot.right - dp(10), plot.top + dp(26), paint);
        }

        private void drawGrid(Canvas canvas, RectF plot) {
            paint.setStyle(Paint.Style.STROKE);
            paint.setStrokeWidth(1.0f);
            for (int i = 0; i <= 4; i++) {
                float y = plot.top + plot.height() * i / 4.0f;
                paint.setColor(i == 2 ? Color.rgb(88, 96, 112) : Color.rgb(38, 43, 52));
                canvas.drawLine(plot.left, y, plot.right, y, paint);
            }
            paint.setColor(Color.rgb(34, 39, 48));
            for (int i = 1; i < 8; i++) {
                float x = plot.left + plot.width() * i / 8.0f;
                canvas.drawLine(x, plot.top, x, plot.bottom, paint);
            }
            paint.setStrokeWidth(2.0f);
            paint.setColor(COLOR_STROKE);
            canvas.drawRoundRect(plot, dp(8), dp(8), paint);

            paint.setStrokeWidth(2.0f);
            paint.setColor(Color.rgb(112, 122, 142));
            float cy = plot.centerY();
            canvas.drawLine(plot.left + dp(4), cy, plot.right - dp(4), cy, paint);
        }

        private float waveScale(int loop) {
            if (!waveAutoScale) {
                return manualWaveScales[loop];
            }
            return autoScale(loop);
        }

        private float autoScale(int loop) {
            float max = MIN_WAVE_SCALE;
            for (int i = 0; i < WAVE_POINTS; i++) {
                max = Math.max(max, Math.abs(waveTargets[loop][i]));
                max = Math.max(max, Math.abs(waveActuals[loop][i]));
            }
            return clampWaveScale(max * 1.18f);
        }

        private void drawWave(Canvas canvas, RectF plot, float[] values, float scale, int color, float strokeWidth) {
            Path path = new Path();
            for (int i = 0; i < WAVE_POINTS; i++) {
                float x = plot.left + plot.width() * i / (WAVE_POINTS - 1.0f);
                float normalized = values[i] / scale;
                if (normalized > 1.0f) {
                    normalized = 1.0f;
                } else if (normalized < -1.0f) {
                    normalized = -1.0f;
                }
                float y = plot.centerY() - normalized * plot.height() * 0.46f;
                if (i == 0) {
                    path.moveTo(x, y);
                } else {
                    path.lineTo(x, y);
                }
            }
            paint.setStyle(Paint.Style.STROKE);
            paint.setStrokeWidth(strokeWidth);
            paint.setColor(color);
            canvas.drawPath(path, paint);
        }

        private void drawLatestMarkers(Canvas canvas, RectF plot, float scale) {
            if (waveSampleCounts[loop] <= 0) {
                return;
            }
            float x = plot.right - dp(8);
            drawMarker(canvas, x, valueToY(plot, latestWaveTargets[loop], scale), COLOR_AMBER);
            drawMarker(canvas, x - dp(12), valueToY(plot, latestWaveActuals[loop], scale), COLOR_BLUE);
        }

        private void drawMarker(Canvas canvas, float x, float y, int color) {
            paint.setStyle(Paint.Style.FILL);
            paint.setColor(color);
            canvas.drawCircle(x, y, dp(4), paint);
            paint.setStyle(Paint.Style.STROKE);
            paint.setStrokeWidth(2.0f);
            paint.setColor(Color.rgb(8, 10, 14));
            canvas.drawCircle(x, y, dp(5), paint);
        }

        private float valueToY(RectF plot, float value, float scale) {
            float normalized = value / scale;
            if (normalized > 1.0f) {
                normalized = 1.0f;
            } else if (normalized < -1.0f) {
                normalized = -1.0f;
            }
            return plot.centerY() - normalized * plot.height() * 0.46f;
        }
    }

    private static final class JoystickView extends View {
        private final Paint paint = new Paint(Paint.ANTI_ALIAS_FLAG);
        private final String label;
        private OnMoveListener listener;
        private float knobX;
        private float knobY;

        JoystickView(Activity activity, String label) {
            super(activity);
            this.label = label;
            setMinimumHeight(activity.getResources().getDisplayMetrics().densityDpi);
        }

        void setOnMoveListener(OnMoveListener listener) {
            this.listener = listener;
        }

        @Override
        protected void onDraw(Canvas canvas) {
            super.onDraw(canvas);
            float width = getWidth();
            float height = getHeight();
            float cx = width * 0.5f;
            float cy = height * 0.53f;
            float radius = Math.min(width, height) * 0.38f;
            float knobRadius = radius * 0.31f;

            paint.setStyle(Paint.Style.FILL);
            paint.setShader(new RadialGradient(cx, cy, radius * 1.15f,
                new int[] { Color.rgb(78, 83, 91), Color.rgb(29, 33, 40), Color.rgb(8, 10, 13) },
                new float[] { 0.0f, 0.62f, 1.0f },
                Shader.TileMode.CLAMP));
            canvas.drawCircle(cx, cy, radius * 1.05f, paint);
            paint.setShader(null);

            RectF outer = new RectF(cx - radius, cy - radius, cx + radius, cy + radius);
            for (int i = 0; i < 4; i++) {
                paint.setStyle(Paint.Style.FILL);
                paint.setColor(i % 2 == 0 ? Color.rgb(31, 35, 42) : Color.rgb(25, 29, 36));
                canvas.drawArc(outer, -45.0f + i * 90.0f, 86.0f, true, paint);
            }

            paint.setStyle(Paint.Style.STROKE);
            paint.setStrokeWidth(3.0f);
            paint.setColor(Color.rgb(7, 9, 12));
            canvas.drawCircle(cx, cy, radius * 1.06f, paint);

            paint.setStrokeWidth(2.0f);
            paint.setColor(Color.rgb(87, 93, 102));
            canvas.drawCircle(cx, cy, radius * 0.96f, paint);

            paint.setStrokeWidth(3.0f);
            paint.setColor(COLOR_PURPLE);
            canvas.drawCircle(cx, cy, radius * 0.52f, paint);

            paint.setStrokeWidth(1.5f);
            paint.setColor(Color.rgb(46, 51, 59));
            canvas.drawCircle(cx, cy, radius * 0.72f, paint);

            paint.setStrokeWidth(3.0f);
            paint.setColor(Color.rgb(11, 13, 17));
            canvas.drawLine(cx - radius * 0.7f, cy - radius * 0.7f, cx + radius * 0.7f, cy + radius * 0.7f, paint);
            canvas.drawLine(cx + radius * 0.7f, cy - radius * 0.7f, cx - radius * 0.7f, cy + radius * 0.7f, paint);

            drawTriangle(canvas, cx, cy - radius * 0.64f, 0.0f, radius * 0.12f);
            drawTriangle(canvas, cx + radius * 0.64f, cy, 90.0f, radius * 0.12f);
            drawTriangle(canvas, cx, cy + radius * 0.64f, 180.0f, radius * 0.12f);
            drawTriangle(canvas, cx - radius * 0.64f, cy, 270.0f, radius * 0.12f);

            paint.setStyle(Paint.Style.FILL);
            float kx = cx + knobX * radius;
            float ky = cy + knobY * radius;
            paint.setColor(Color.rgb(9, 11, 15));
            canvas.drawCircle(kx, ky + knobRadius * 0.13f, knobRadius * 1.08f, paint);

            paint.setShader(new RadialGradient(kx - knobRadius * 0.25f, ky - knobRadius * 0.28f, knobRadius * 1.2f,
                new int[] { Color.rgb(126, 132, 140), Color.rgb(42, 45, 53), Color.rgb(7, 8, 11) },
                new float[] { 0.0f, 0.48f, 1.0f },
                Shader.TileMode.CLAMP));
            canvas.drawCircle(kx, ky, knobRadius, paint);
            paint.setShader(null);

            paint.setStyle(Paint.Style.STROKE);
            paint.setStrokeWidth(3.0f);
            paint.setColor(COLOR_PURPLE);
            canvas.drawCircle(kx, ky, knobRadius * 1.16f, paint);
            paint.setStrokeWidth(1.5f);
            paint.setColor(Color.rgb(198, 185, 255));
            canvas.drawCircle(kx, ky, knobRadius * 0.83f, paint);

            paint.setStyle(Paint.Style.FILL);
            paint.setColor(Color.rgb(237, 233, 249));
            paint.setTextAlign(Paint.Align.CENTER);
            paint.setTextSize(34.0f);
            paint.setTypeface(Typeface.DEFAULT_BOLD);
            canvas.drawText(label, cx, cy - radius - 10.0f, paint);
        }

        private void drawTriangle(Canvas canvas, float cx, float cy, float degrees, float size) {
            Path path = new Path();
            path.moveTo(0.0f, -size);
            path.lineTo(size * 0.82f, size * 0.72f);
            path.lineTo(-size * 0.82f, size * 0.72f);
            path.close();
            canvas.save();
            canvas.translate(cx, cy);
            canvas.rotate(degrees);
            paint.setStyle(Paint.Style.FILL);
            paint.setColor(COLOR_PURPLE);
            canvas.drawPath(path, paint);
            canvas.restore();
        }

        @Override
        public boolean onTouchEvent(MotionEvent event) {
            float width = getWidth();
            float height = getHeight();
            float cx = width * 0.5f;
            float cy = height * 0.53f;
            float radius = Math.min(width, height) * 0.38f;

            if (event.getAction() == MotionEvent.ACTION_UP || event.getAction() == MotionEvent.ACTION_CANCEL) {
                knobX = 0.0f;
                knobY = 0.0f;
                if (listener != null) {
                    listener.onMove(0.0f, 0.0f, true);
                }
                invalidate();
                return true;
            }

            float dx = (event.getX() - cx) / radius;
            float dy = (event.getY() - cy) / radius;
            float length = (float)Math.sqrt(dx * dx + dy * dy);
            if (length > 1.0f) {
                dx /= length;
                dy /= length;
            }
            knobX = dx;
            knobY = dy;
            if (listener != null) {
                listener.onMove(knobX, knobY, false);
            }
            invalidate();
            return true;
        }
    }

    private interface OnMoveListener {
        void onMove(float x, float y, boolean released);
    }

    private static final class TelemetryDisplayView extends View {
        private final Paint paint = new Paint(Paint.ANTI_ALIAS_FLAG);
        private final String[] labels = {"TEMPERATURE", "HUMIDITY", "WEIGHT", "HX711 RAW"};
        private final String[] icons = {"T", "H", "W", "X"};
        private final String[] values = {"--.- C", "-- %", "-- g", "----"};

        TelemetryDisplayView(Activity activity) {
            super(activity);
            setMinimumHeight(activity.getResources().getDisplayMetrics().densityDpi / 2);
        }

        void setTelemetry(String temp, String humi, String weight, String adc) {
            values[0] = valuePart(temp);
            values[1] = valuePart(humi);
            values[2] = valuePart(weight);
            values[3] = valuePart(adc);
            invalidate();
        }

        private String valuePart(String text) {
            int index = text.indexOf(':');
            if (index >= 0 && index + 1 < text.length()) {
                return text.substring(index + 1).trim();
            }
            return text;
        }

        @Override
        protected void onDraw(Canvas canvas) {
            super.onDraw(canvas);
            float w = getWidth();
            float h = getHeight();
            float pad = Math.min(w, h) * 0.08f;
            RectF outer = new RectF(pad, pad * 0.5f, w - pad, h - pad * 0.5f);

            paint.setStyle(Paint.Style.FILL);
            paint.setColor(Color.rgb(4, 5, 8));
            canvas.drawRoundRect(outer, 14.0f, 14.0f, paint);

            RectF inner = new RectF(outer.left + 8.0f, outer.top + 8.0f, outer.right - 8.0f, outer.bottom - 8.0f);
            paint.setShader(new RadialGradient(inner.centerX(), inner.top, inner.width() * 0.9f,
                new int[] { Color.rgb(24, 28, 36), Color.rgb(10, 12, 17), Color.rgb(5, 6, 9) },
                new float[] { 0.0f, 0.55f, 1.0f },
                Shader.TileMode.CLAMP));
            canvas.drawRoundRect(inner, 8.0f, 8.0f, paint);
            paint.setShader(null);

            paint.setStyle(Paint.Style.STROKE);
            paint.setStrokeWidth(2.0f);
            paint.setColor(Color.rgb(58, 64, 76));
            canvas.drawRoundRect(inner, 8.0f, 8.0f, paint);

            float rowH = inner.height() / 4.0f;
            for (int i = 1; i < 4; i++) {
                float y = inner.top + rowH * i;
                paint.setStyle(Paint.Style.STROKE);
                paint.setStrokeWidth(1.0f);
                paint.setColor(Color.rgb(42, 47, 56));
                canvas.drawLine(inner.left + 10.0f, y, inner.right - 10.0f, y, paint);
            }

            for (int i = 0; i < 4; i++) {
                float cy = inner.top + rowH * (i + 0.5f);
                drawIcon(canvas, inner.left + 28.0f, cy, icons[i]);

                paint.setStyle(Paint.Style.FILL);
                paint.setShader(null);
                paint.setTextAlign(Paint.Align.LEFT);
                paint.setTypeface(Typeface.DEFAULT_BOLD);
                paint.setTextSize(24.0f);
                paint.setColor(Color.rgb(226, 230, 239));
                canvas.drawText(labels[i], inner.left + 58.0f, cy + 8.0f, paint);

                paint.setTextAlign(Paint.Align.RIGHT);
                paint.setTextSize(26.0f);
                paint.setColor(Color.rgb(171, 131, 255));
                canvas.drawText(values[i], inner.right - 22.0f, cy + 9.0f, paint);
            }
        }

        private void drawIcon(Canvas canvas, float cx, float cy, String text) {
            paint.setStyle(Paint.Style.FILL);
            paint.setColor(Color.rgb(31, 34, 43));
            canvas.drawRoundRect(new RectF(cx - 12.0f, cy - 12.0f, cx + 12.0f, cy + 12.0f), 4.0f, 4.0f, paint);

            paint.setStyle(Paint.Style.STROKE);
            paint.setStrokeWidth(1.5f);
            paint.setColor(Color.rgb(171, 131, 255));
            canvas.drawRoundRect(new RectF(cx - 12.0f, cy - 12.0f, cx + 12.0f, cy + 12.0f), 4.0f, 4.0f, paint);

            paint.setStyle(Paint.Style.FILL);
            paint.setTextAlign(Paint.Align.CENTER);
            paint.setTypeface(Typeface.DEFAULT_BOLD);
            paint.setTextSize(17.0f);
            paint.setColor(Color.rgb(230, 222, 255));
            canvas.drawText(text, cx, cy + 6.0f, paint);
        }
    }
}
