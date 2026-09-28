package com.makeyourpet.chicaserver;

import androidx.appcompat.app.ActionBar;
import androidx.appcompat.app.AppCompatActivity;
import androidx.appcompat.app.AlertDialog;
import android.content.Intent;
import android.content.res.Configuration;
import android.graphics.Typeface;
import android.net.wifi.WifiManager;
import android.net.Uri;
import android.os.Bundle;
import android.text.format.Formatter;
import android.view.View;
import android.view.ViewGroup;
import android.widget.FrameLayout;
import android.widget.EditText;
import android.widget.Button;
import com.makeyourpet.chicaserver.control.ChicaController;
import com.makeyourpet.chicaserver.control.OnnxLocomotionRunner;
import com.makeyourpet.chicaserver.hardware.ServoBackendFactory;
import com.makeyourpet.chicaserver.protocol.OriginalTcpControlServer;
import android.util.Log;

public final class FullscreenActivity extends AppCompatActivity {
    private static final int IMMERSIVE_FLAGS = 0x1307;
    private static final int ACTIVE_BUTTON_TEXT = 0xff00ffff;
    private static final int INACTIVE_BUTTON_TEXT = 0xff000000;

    private ChicaController controller;
    private OriginalTcpControlServer server;
    private OriginalOrientationSensor orientationSensor;
    private ConfigStore configStore;
    private FrameLayout rootView;
    private InfoSurfaceView infoSurfaceView;
    private View cameraView;
    private EditText configEditor;
    private AlertDialog configDialog;
    private String configSummary = "";
    private final Runnable refreshStatus = new Runnable() {
        @Override
        public void run() {
            if (infoSurfaceView == null) return;
            refreshNow();
            infoSurfaceView.postDelayed(this, 83);
        }
    };

    @Override
    protected void attachBaseContext(android.content.Context newBase) {
        try {
            Configuration configuration = new Configuration(newBase.getResources().getConfiguration());
            configuration.fontScale = 1.0f;
            applyOverrideConfiguration(configuration);
        } catch (Exception ignored) {
        }
        super.attachBaseContext(newBase);
    }

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_fullscreen);
        ViewGroup content = findViewById(android.R.id.content);
        rootView = (FrameLayout) content.getChildAt(0);
        hideSystemUi();

        configStore = new ConfigStore(this);
        String config = configStore.load();
        controller = new ChicaController(ServoBackendFactory.open(this, config), config);
        try {
            OnnxLocomotionRunner onnxRunner = new OnnxLocomotionRunner(this, "hexapod_policy.onnx");
            controller.setOnnxLocomotionRunner(onnxRunner);
            Log.i("ChicaServer", "ONNX locomotion policy hexapod_policy.onnx initialized successfully");
        } catch (Throwable t) {
            Log.w("ChicaServer", "ONNX locomotion initialization note: " + t.getMessage());
        }
        orientationSensor = new OriginalOrientationSensor(this, controller);
        orientationSensor.start();
        configSummary = controller.configSummary(config);
        infoSurfaceView = findViewById(R.id.infoSurfaceView);
        cameraView = findViewById(R.id.cameraView);
        configEditor = new EditText(this);
        configEditor.setTextSize(0, getResources().getDimension(R.dimen.configEditTextTextSize));
        configEditor.setHeight((int) getResources().getDimension(R.dimen.configEditTextHeight));
        configEditor.setBackgroundColor(0xff000000);
        configEditor.setTextColor(0xff00ff00);
        configEditor.setAllCaps(true);
        configEditor.setTypeface(Typeface.MONOSPACE);
        configEditor.setHorizontalScrollBarEnabled(true);
        configEditor.setVerticalScrollBarEnabled(true);

        findViewById(R.id.buttonBlock).setOnClickListener(view -> {
            controller.requestBlock();
        });
        findViewById(R.id.buttonTorque).setOnClickListener(view -> {
            controller.requestTorque();
        });
        findViewById(R.id.buttonConfig).setOnClickListener(view -> showConfigDialog());
        findViewById(R.id.buttonCamera).setOnClickListener(view -> toggleCameraView());
        findViewById(R.id.buttonPrivacyPolicy).setOnClickListener(view -> openPrivacyPolicy());
        findViewById(R.id.buttonExit).setOnClickListener(view -> {
            finish();
            System.exit(0);
        });

        server = new OriginalTcpControlServer(controller);
        server.start();
        refreshNow();
        infoSurfaceView.post(refreshStatus);
    }

    @Override
    protected void onDestroy() {
        if (infoSurfaceView != null) infoSurfaceView.removeCallbacks(refreshStatus);
        if (configDialog != null && configDialog.isShowing()) {
            try {
                configDialog.dismiss();
            } catch (Exception ignored) {
            }
        }
        if (orientationSensor != null) orientationSensor.stop();
        if (server != null) server.stop();
        if (controller != null) controller.close();
        super.onDestroy();
    }

    private void refreshNow() {
        if (infoSurfaceView != null) {
            infoSurfaceView.setSurfaceStatus(controller.surfaceStatus(ipAddress()));
        }
        setStateButtonTextColor(R.id.buttonBlock, controller.isBlockMode());
        setStateButtonTextColor(R.id.buttonTorque, controller.isRelayEnabled());
    }

    private void setStateButtonTextColor(int id, boolean active) {
        View view = findViewById(id);
        if (view instanceof Button) {
            ((Button) view).setTextColor(active ? ACTIVE_BUTTON_TEXT : INACTIVE_BUTTON_TEXT);
        }
    }

    private void hideSystemUi() {
        ActionBar actionBar = getSupportActionBar();
        if (actionBar != null) actionBar.hide();
        if (rootView != null) {
            rootView.setSystemUiVisibility(IMMERSIVE_FLAGS);
        } else {
            getWindow().getDecorView().setSystemUiVisibility(IMMERSIVE_FLAGS);
        }
    }

    private void showConfigDialog() {
        configEditor.setText(configStore.load());
        if (configDialog == null) {
            configDialog = new AlertDialog.Builder(this)
                    .setTitle(R.string.app_name)
                    .setMessage(R.string.version_name)
                    .setView(configEditor)
                    .setPositiveButton("Save", (dialog, which) -> {
                        configStore.save(configEditor.getText().toString());
                        configSummary = controller.configSummary(configStore.load());
                        hideSystemUi();
                        refreshNow();
                    })
                    .setNegativeButton("Cancel", (dialog, which) -> {
                        hideSystemUi();
                        refreshNow();
                    })
                    .create();
        }
        configDialog.show();
    }

    private void toggleCameraView() {
        if (cameraView == null || infoSurfaceView == null || controller == null) return;
        if (!controller.isCameraEnabled()) {
            controller.setCameraEnabled(true);
            cameraView.setTop(cameraView.getBottom() - ((cameraView.getWidth() * 240) / 640));
            infoSurfaceView.setBottom(cameraView.getTop());
        } else {
            System.out.println("fps: 1");
        }
        refreshNow();
    }

    private void openPrivacyPolicy() {
        Intent intent = Intent.makeMainSelectorActivity(
                Intent.ACTION_MAIN,
                Intent.CATEGORY_APP_BROWSER);
        intent.setData(Uri.parse("https://www.makeyourpet.com/privacy-policy"));
        startActivity(intent);
    }

    private String ipAddress() {
        try {
            WifiManager wifi = (WifiManager) getApplicationContext().getSystemService(WIFI_SERVICE);
            if (wifi != null) return Formatter.formatIpAddress(wifi.getConnectionInfo().getIpAddress());
        } catch (Exception ignored) {
        }
        return "0.0.0.0";
    }
}
