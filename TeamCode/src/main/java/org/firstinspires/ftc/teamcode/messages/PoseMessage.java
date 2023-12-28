package org.firstinspires.ftc.teamcode.messages;

import com.acmerobotics.roadrunner.Pose2d;

public final class PoseMessage {
    public final long timestamp;
    public final double x;
    public final double y;
    public final double heading;

    public PoseMessage(Pose2d pose) {
        this.timestamp = System.nanoTime();
        this.x = pose.position.x;
        this.y = pose.position.y;
        this.heading = pose.heading.toDouble();
    }
}

