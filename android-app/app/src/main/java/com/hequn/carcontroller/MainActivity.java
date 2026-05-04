package com.hequn.carcontroller;

import android.Manifest;
import android.app.Activity;
import android.bluetooth.BluetoothAdapter;
import android.bluetooth.BluetoothDevice;
import android.bluetooth.BluetoothSocket;
import android.content.pm.PackageManager;
import android.graphics.Color;
import android.graphics.Typeface;
import android.os.Build;
import android.os.Bundle;
import android.view.Gravity;
import android.view.MotionEvent;
import android.view.View;
import android.widget.*;
import java.io.IOException;
import java.io.OutputStream;
import java.util.ArrayList;
import java.util.Set;
import java.util.UUID;

public class MainActivity extends Activity {
    private static final UUID SPP_UUID = UUID.fromString("00001101-0000-1000-8000-00805F9B34FB");
    private static final int REQ_BLUETOOTH_CONNECT = 1;
    private BluetoothAdapter adapter;
    private BluetoothSocket socket;
    private OutputStream output;
    private TextView status;
    private TextView log;
    private LinearLayout deviceList;

    @Override
    protected void onCreate(Bundle b) {
        super.onCreate(b);
        adapter = BluetoothAdapter.getDefaultAdapter();
        if (!hasBluetoothPermission()) {
            requestPermissions(new String[]{Manifest.permission.BLUETOOTH_CONNECT}, REQ_BLUETOOTH_CONNECT);
        }
        buildUi();
    }

    private void buildUi() {
        ScrollView scroll = new ScrollView(this);
        LinearLayout root = col();
        root.setPadding(dp(18), dp(18), dp(18), dp(24));
        root.setBackgroundColor(Color.rgb(7, 17, 31));
        scroll.addView(root);

        TextView mark = text("STM32 BALANCE CAR", 12, Color.rgb(56, 189, 248), true);
        mark.setLetterSpacing(0.14f);
        root.addView(mark);

        TextView title = text("贺群的小车控制器", 28, Color.rgb(229, 237, 247), true);
        title.setPadding(0, dp(4), 0, dp(2));
        root.addView(title);

        TextView sub = text("经典蓝牙 SPP / HC-05 / HC-06", 14, Color.rgb(144, 164, 189), false);
        root.addView(sub);

        status = pill("未连接");
        status.setGravity(Gravity.CENTER);
        LinearLayout.LayoutParams sp = new LinearLayout.LayoutParams(-1, dp(44));
        sp.setMargins(0, dp(14), 0, dp(10));
        root.addView(status, sp);

        Button refresh = btn("刷新已配对蓝牙", Color.rgb(14, 165, 233));
        refresh.setOnClickListener(v -> showPairedDevices());
        root.addView(refresh, lp());

        deviceList = col();
        root.addView(deviceList);

        LinearLayout mode = row();
        mode.addView(cmdButton("手动", "M", Color.rgb(30, 41, 59)), weight());
        mode.addView(cmdButton("跟随", "U", Color.rgb(21, 94, 117)), weight());
        mode.addView(cmdButton("避障", "A", Color.rgb(113, 63, 18)), weight());
        root.addView(card(mode), lp());

        LinearLayout pad = col();
        LinearLayout r1 = row();
        r1.addView(space(), weight());
        r1.addView(cmdButton("前进", "F", Color.rgb(22, 101, 52)), weight());
        r1.addView(space(), weight());
        pad.addView(r1);
        LinearLayout r2 = row();
        r2.addView(cmdButton("左转", "L", Color.rgb(30, 41, 59)), weight());
        r2.addView(cmdButton("停止", "S", Color.rgb(185, 28, 28)), weight());
        r2.addView(cmdButton("右转", "R", Color.rgb(30, 41, 59)), weight());
        pad.addView(r2);
        LinearLayout r3 = row();
        r3.addView(space(), weight());
        r3.addView(cmdButton("后退", "B", Color.rgb(120, 53, 15)), weight());
        r3.addView(space(), weight());
        pad.addView(r3);
        root.addView(card(pad), lp());

        log = text("", 12, Color.rgb(203, 213, 225), false);
        log.setTypeface(Typeface.MONOSPACE);
        log.setMinHeight(dp(110));
        log.setPadding(dp(12), dp(10), dp(12), dp(10));
        log.setBackgroundColor(Color.rgb(3, 7, 18));
        root.addView(log, lp());

        TextView foot = text("Designed for 贺群 · 小霸王Lite", 12, Color.rgb(111, 126, 146), false);
        foot.setGravity(Gravity.CENTER);
        foot.setPadding(0, dp(12), 0, 0);
        root.addView(foot);
        setContentView(scroll);
        showPairedDevices();
    }

    private void showPairedDevices() {
        deviceList.removeAllViews();
        if (adapter == null) {
            addLog("手机不支持蓝牙");
            return;
        }
        if (!hasBluetoothPermission()) {
            addLog("请先允许蓝牙权限，然后点刷新。");
            return;
        }
        Set<BluetoothDevice> bonded = adapter.getBondedDevices();
        if (bonded == null || bonded.isEmpty()) {
            addLog("没有已配对设备。先到系统蓝牙里配对 HC-05/HC-06。");
            return;
        }
        ArrayList<BluetoothDevice> list = new ArrayList<>(bonded);
        for (BluetoothDevice d : list) {
            Button b = btn("连接 " + d.getName() + "  " + d.getAddress(), Color.rgb(37, 99, 235));
            b.setOnClickListener(v -> connect(d));
            deviceList.addView(b, lp());
        }
    }

    private void connect(BluetoothDevice d) {
        if (!hasBluetoothPermission()) {
            addLog("请先允许蓝牙权限。");
            return;
        }
        new Thread(() -> {
            try {
                if (socket != null) socket.close();
                socket = d.createRfcommSocketToServiceRecord(SPP_UUID);
                socket.connect();
                output = socket.getOutputStream();
                runOnUiThread(() -> {
                    status.setText("已连接：" + d.getName());
                    status.setTextColor(Color.rgb(187, 247, 208));
                    addLog("已连接 " + d.getName());
                    send("M");
                });
            } catch (Exception e) {
                runOnUiThread(() -> addLog("连接失败：" + e.getMessage()));
            }
        }).start();
    }

    @Override
    public void onRequestPermissionsResult(int requestCode, String[] permissions, int[] grantResults) {
        super.onRequestPermissionsResult(requestCode, permissions, grantResults);
        if (requestCode == REQ_BLUETOOTH_CONNECT) {
            if (grantResults.length > 0 && grantResults[0] == PackageManager.PERMISSION_GRANTED) {
                addLog("蓝牙权限已允许");
                showPairedDevices();
            } else {
                addLog("蓝牙权限未允许，无法连接模块。");
            }
        }
    }

    private boolean hasBluetoothPermission() {
        return Build.VERSION.SDK_INT < 31 ||
            checkSelfPermission(Manifest.permission.BLUETOOTH_CONNECT) == PackageManager.PERMISSION_GRANTED;
    }

    private Button cmdButton(String label, String cmd, int color) {
        Button b = btn(label, color);
        if ("F".equals(cmd) || "B".equals(cmd) || "L".equals(cmd) || "R".equals(cmd)) {
            b.setOnTouchListener((v, event) -> {
                if (event.getAction() == MotionEvent.ACTION_DOWN) {
                    send(cmd);
                    return true;
                }
                if (event.getAction() == MotionEvent.ACTION_UP
                    || event.getAction() == MotionEvent.ACTION_CANCEL) {
                    send("S");
                    return true;
                }
                return true;
            });
        } else {
            b.setOnClickListener(v -> send(cmd));
        }
        return b;
    }

    private void send(String cmd) {
        try {
            if (output == null) {
                addLog("未连接，无法发送 " + cmd);
                return;
            }
            output.write(cmd.getBytes());
            output.flush();
            addLog("发送：" + cmd);
        } catch (IOException e) {
            addLog("发送失败：" + e.getMessage());
        }
    }

    private void addLog(String s) {
        log.append(s + "\n");
    }

    private LinearLayout col() {
        LinearLayout l = new LinearLayout(this);
        l.setOrientation(LinearLayout.VERTICAL);
        return l;
    }

    private LinearLayout row() {
        LinearLayout l = new LinearLayout(this);
        l.setOrientation(LinearLayout.HORIZONTAL);
        l.setGravity(Gravity.CENTER);
        return l;
    }

    private LinearLayout card(View child) {
        LinearLayout l = col();
        l.setPadding(dp(12), dp(12), dp(12), dp(12));
        l.setBackgroundColor(Color.rgb(15, 23, 42));
        l.addView(child);
        return l;
    }

    private TextView text(String s, int size, int color, boolean bold) {
        TextView t = new TextView(this);
        t.setText(s);
        t.setTextSize(size);
        t.setTextColor(color);
        if (bold) t.setTypeface(Typeface.DEFAULT, Typeface.BOLD);
        return t;
    }

    private TextView pill(String s) {
        TextView t = text(s, 14, Color.rgb(144, 164, 189), false);
        t.setBackgroundColor(Color.rgb(15, 23, 42));
        return t;
    }

    private Button btn(String s, int color) {
        Button b = new Button(this);
        b.setText(s);
        b.setTextColor(Color.WHITE);
        b.setTextSize(16);
        b.setTypeface(Typeface.DEFAULT, Typeface.BOLD);
        b.setBackgroundColor(color);
        return b;
    }

    private TextView space() {
        TextView v = new TextView(this);
        v.setText("");
        return v;
    }

    private LinearLayout.LayoutParams lp() {
        LinearLayout.LayoutParams p = new LinearLayout.LayoutParams(-1, -2);
        p.setMargins(0, dp(8), 0, dp(8));
        return p;
    }

    private LinearLayout.LayoutParams weight() {
        LinearLayout.LayoutParams p = new LinearLayout.LayoutParams(0, dp(62), 1);
        p.setMargins(dp(5), dp(5), dp(5), dp(5));
        return p;
    }

    private int dp(int v) {
        return (int)(v * getResources().getDisplayMetrics().density + 0.5f);
    }
}
