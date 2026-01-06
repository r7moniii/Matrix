package org.firstinspires.ftc.teamcode.Testing;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public class Yes extends OpMode {

    Follower follower;
    @Override
    public void init() {
    }

    @Override
    public void loop() {
        follower.update();
        Constants.heading = follower.getHeading();
    }
}
