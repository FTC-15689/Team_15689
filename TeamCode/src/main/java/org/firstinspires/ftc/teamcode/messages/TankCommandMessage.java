package org.firstinspires.ftc.teamcode.messages;

public final class TankCommandMessage {
    public final long timestamp;
    public final double voltage;
    public final double leftPower;
    public final double rightPower;

    public TankCommandMessage(double voltage, double leftPower, double rightPower) {
        this.timestamp = System.nanoTime();
        this.voltage = voltage;
        this.leftPower = leftPower;
        this.rightPower = rightPower;
    }
}
