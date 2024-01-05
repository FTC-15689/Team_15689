package org.firstinspires.ftc.teamcode.drive.utils;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.ArrayList;
import java.util.List;

public class ActionHandler {
    private final LinearOpMode opMode;
    private boolean isBusy = false;
    private List<Action> actions = new ArrayList<>();
    private final FtcDashboard dash = FtcDashboard.getInstance();

    public ActionHandler(LinearOpMode myOpMode) {
        opMode = myOpMode;
    }

    private void periodic() {
        while (true) {
            TelemetryPacket packet = new TelemetryPacket();

            List<Action> running = new ArrayList<>();
            for (Action act : actions) {
                act.preview(packet.fieldOverlay());
                if (act.run(packet)) {
                    running.add(act);
                }
            }
            actions = running;
            isBusy = actions.size() > 0;

            dash.sendTelemetryPacket(packet);
        }
    }
    public void followAsync(Action action) {
        actions.add(action);
    }
    public boolean isBusy() {
        return isBusy;
    }
}
