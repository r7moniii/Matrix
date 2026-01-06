package org.firstinspires.ftc.teamcode;

import com.pedropathing.control.PIDFController;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Turret {
    double TICKS_PER_DEGREE;
    double X, Y, currentHeading;
    HardwareMap hardwareMap;
    PIDFController contoller;
    Turret(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
    }
    DcMotor motor;
    void init() {
        motor = hardwareMap.get(DcMotor.class, "motor");
    }

    void update() {
        double currentPosition = motor.getCurrentPosition() / TICKS_PER_DEGREE;
        double angle = Math.toDegrees(Math.atan2(144 - Y, 144 - X));
        double error = (Constants.heading - 180) + angle + currentPosition;
        double power = contoller.calculate(error);
        motor.setPower(power);
    }
}
