package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

public class LinearSlidePID extends OpMode {
    DcMotor linearRight;
    DcMotor linearLeft;

    double Kp = 0;
    double Ki = 0;
    double Kd = 0;

    double reference = 0;

    double integralSum = 0;

    double lastError = 0;
    double LRP = 0;
    double encoderPosition = linearLeft.getCurrentPosition();



    @Override
    public void init() {
    linearLeft = hardwareMap.get(DcMotor.class, "linearLeft");
    linearRight = hardwareMap.get(DcMotor.class, "linearRight");

    }

    @Override
    public void loop() {
        // Elapsed timer class from SDK, please use it, it's epic
        ElapsedTime timer = new ElapsedTime();
        linearLeft.setPower(LRP);
        linearRight.setPower(LRP);

        while (reference != encoderPosition) {

            // calculate the error
            double error = reference - encoderPosition;

            // rate of change of the error
            double derivative = (error - lastError) / timer.seconds();

            // sum of all error over time
            integralSum = integralSum + (error * timer.seconds());

            LRP = (Kp * error) + (Ki * integralSum) + (Kd * derivative);

            linearLeft.setPower(LRP);
            linearRight.setPower(LRP);

            lastError = error;

            // reset the timer for next time
            timer.reset();

        }

    }
}
