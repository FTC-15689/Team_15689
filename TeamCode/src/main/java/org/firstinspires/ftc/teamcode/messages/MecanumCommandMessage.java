package org.firstinspires.ftc.teamcode.messages;

public final class MecanumCommandMessage {
    public final long timestamp;
    public final double voltage;
    public final double leftFrontPower;
    public final double leftBackPower;
    public final double rightBackPower;
    public final double rightFrontPower;

    public MecanumCommandMessage(double voltage, double leftFrontPower, double leftBackPower, double rightBackPower, double rightFrontPower) {
        this.timestamp = System.nanoTime();
        this.voltage = voltage;
        this.leftFrontPower = leftFrontPower;
        this.leftBackPower = leftBackPower;
        this.rightBackPower = rightBackPower;
        this.rightFrontPower = rightFrontPower;
    }
}
