package org.firstinspires.ftc.teamcode.messages;

import com.acmerobotics.roadrunner.ftc.PositionVelocityPair;

public final class ThreeDeadWheelEncodersMessage {
    public final long timestamp;
    public final PositionVelocityPair par0;
    public final PositionVelocityPair par1;
    public final PositionVelocityPair perp;

    public ThreeDeadWheelEncodersMessage(PositionVelocityPair par0, PositionVelocityPair par1, PositionVelocityPair perp) {
        this.timestamp = System.nanoTime();
        this.par0 = par0;
        this.par1 = par1;
        this.perp = perp;
    }
}