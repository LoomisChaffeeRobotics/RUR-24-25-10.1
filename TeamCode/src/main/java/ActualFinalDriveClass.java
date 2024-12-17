import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp
public class ActualFinalDriveClass extends OpMode {
    IMU imu;
    boolean inited = false;
    DcMotor leftFront;
    DcMotor rightFront;
    DcMotor leftRear;
    DcMotor rightRear;

    @Override
    public void init() {
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        if (imu.initialize(parameters)){
            inited = true;
        }
        leftFront = hardwareMap.get(DcMotorEx.class, "frontLeft");
        leftRear = hardwareMap.get(DcMotorEx.class, "rearLeft");
        rightRear = hardwareMap.get(DcMotorEx.class, "rearRight");
        rightFront = hardwareMap.get(DcMotorEx.class, "frontRight");
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);

    }

    @Override
    public void loop() {
        if(gamepad1.a){
            imu.resetYaw();
        }

        double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;
        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        // Rotate the movement direction counter to the bot's rotation
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;

        leftFront.setPower(frontLeftPower);
        leftRear.setPower(backLeftPower);
        rightFront.setPower(frontRightPower);
        rightRear.setPower(backRightPower);

    telemetry.addData("inited: ", inited);
    telemetry.addData("yaw",imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS)) ;
    telemetry.addData("pitch",imu.getRobotYawPitchRollAngles().getPitch(AngleUnit.RADIANS));
    telemetry.addData("roll",imu.getRobotYawPitchRollAngles().getRoll(AngleUnit.RADIANS));
    telemetry.update();
    }
}
