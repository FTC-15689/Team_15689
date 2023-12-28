package org.firstinspires.ftc.teamcode.messages;

import com.acmerobotics.roadrunner.ftc.PositionVelocityPair;

public final class TwoDeadWheelEncodersMessage {
    public final long timestamp;
    public final PositionVelocityPair par;
    public final PositionVelocityPair perp;

    public TwoDeadWheelEncodersMessage(PositionVelocityPair par, PositionVelocityPair perp) {
        this.timestamp = System.nanoTime();
        this.par = par;
        this.perp = perp;
    }
}
