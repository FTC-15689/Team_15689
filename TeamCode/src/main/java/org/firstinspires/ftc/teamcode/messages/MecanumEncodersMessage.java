package org.firstinspires.ftc.teamcode.messages;

import com.acmerobotics.roadrunner.ftc.PositionVelocityPair;

public final class MecanumEncodersMessage {
    public final long timestamp;
    public final PositionVelocityPair leftFront;
    public final PositionVelocityPair leftBack;
    public final PositionVelocityPair rightBack;
    public final PositionVelocityPair rightFront;

    public MecanumEncodersMessage(PositionVelocityPair leftFront, PositionVelocityPair leftBack, PositionVelocityPair rightBack, PositionVelocityPair rightFront) {
        this.timestamp = System.nanoTime();
        this.leftFront = leftFront;
        this.leftBack = leftBack;
        this.rightBack = rightBack;
        this.rightFront = rightFront;
    }
}
