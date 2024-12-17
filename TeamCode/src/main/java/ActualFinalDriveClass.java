import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.servoWork;

@TeleOp
public class ActualFinalDriveClass extends OpMode {
    servoWork servos;
    IMU imu;
    boolean inited = false;
    SampleMecanumDrive drive;
    boolean rbdepressed = false;

    @Override
    public void init() {
        servos = new servoWork();
        servos.init(hardwareMap); // init servos
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.initialize(parameters);
        drive = new SampleMecanumDrive(hardwareMap);

    }

    @Override
    public void loop() {
        /*
        Gmpd1:
            START: resetYaw();
            LStick: moving FC

        Gmpd2:
            START: resetYaw();
            RStick: Turning
            Rbumper: arm
            LBumber: claw



         */
        if(gamepad1.right_bumper) {
            telemetry.addLine("rb push");
            if (!rbdepressed){
                servos.clawToggle();
                rbdepressed = true;
            }
        } else {
            rbdepressed = false;
        }
        if(gamepad1.start){
            imu.resetYaw();
        }
        if (gamepad1.dpad_up){
            servos.clawManual( -0.01);
        }
        if (gamepad1.dpad_down){
            servos.clawManual(0.01);
        }


        double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x; //Change to Gmpd2 later
        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        // Rotate the movement direction counter to the bot's rotation
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;

        drive.setMotorPowers(frontLeftPower,backLeftPower,backRightPower,frontRightPower);

    telemetry.addData("inited: ", inited);
    telemetry.addData("yaw",imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS)) ;
    telemetry.addData("pitch",imu.getRobotYawPitchRollAngles().getPitch(AngleUnit.RADIANS));
    telemetry.addData("roll",imu.getRobotYawPitchRollAngles().getRoll(AngleUnit.RADIANS));
    telemetry.update();
    }
}
