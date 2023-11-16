package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Disabled
@Config
public class config {
    public final double SPEED = 0.3d;
    public final double BOOST = 0.7d;

    public final double ARM_SPEED = 0.6d;
    public final double ARM_BOOST = 0.8d;

    public final double kP = 0.1d;
    public final double kI = 0.01d;
    public final double kD = 0.001d;
}
