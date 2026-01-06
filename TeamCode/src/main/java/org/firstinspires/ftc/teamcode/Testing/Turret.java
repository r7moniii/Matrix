package org.firstinspires.ftc.teamcode.Testing;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Turret extends Constants.turret {
    double TICKS_PER_DEGREE;
    HardwareMap hardwareMap;
    PIDController controller = new PIDController(kp, ki, kd);
    Turret(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
    }
    DcMotor motor;
    void init() {
        motor = hardwareMap.get(DcMotor.class, "motor");
    }

    void update() {
        controller.setPID(kp, ki, kd);

        if(Constants.localization.heading < 0) {
            Constants.localization.heading += 360;
        }

        double currentPosition = motor.getCurrentPosition() / TICKS_PER_DEGREE;

        double angle = Math.toDegrees(Math.atan2(144 - Constants.localization.Y, 144 - Constants.localization.X));

        double error = (Constants.localization.heading - 180) + angle + currentPosition;

        double power = controller.calculate(error);

        motor.setPower(power);
    }
}
