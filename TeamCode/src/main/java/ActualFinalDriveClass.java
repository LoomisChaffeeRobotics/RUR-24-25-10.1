import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
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
    double armLPos;
    boolean ydepressed = false;
    boolean adepressed = false;
    double ADP;
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
            RStick: Turning
        Gmpd2:
            START: resetYaw();
            RStick: Turning (Low speed)
            Dpad: arm
            RBumper: claw
            Triggers: Linear Slide
         */
        if (gamepad1.y) {
            servos.extenderForward();
        } else if (gamepad1.a) {
            servos.extenderBack();
        } else {
            servos.extenderNeutral();
        }
        if (gamepad1.right_bumper) {
            if (!rbdepressed) {
                servos.clawToggle();
                rbdepressed = true;
            }
        } else {
            rbdepressed = false;
        }
        if (gamepad1.start) {
            imu.resetYaw();
        }
        if (gamepad2.start) {
            imu.resetYaw();
        }
        if (gamepad1.dpad_up) {
            servos.armUp();
        }

        servos.armUpdate();

        telemetry.addData("armDownPercent", servos.armDownPercent);

        if (gamepad1.dpad_down) {
            servos.armDown();
        }
        if (gamepad1.dpad_right) {
            servos.arm45();
        }
        if (gamepad1.dpad_left){
            servos.armSpecimen();
        }
        if (gamepad1.x){
            servos.tiltTilt(1d);
        } else if (gamepad1.b){
            servos.tiltTilt(-1d);
        } else {
            servos.tiltTilt(0d);
        }

        servos.liftLift(gamepad1.right_trigger - gamepad1.left_trigger+0.12);

        double speedFactor = Math.min(1.3 - gamepad2.right_trigger, 1); // so driver can slow down

        double y = -gamepad2.left_stick_y; // Remember, Y stick value is reversed
        double x = gamepad2.left_stick_x; //dylan is my bf // If your going to write my name, bother to capitalize it. - Dylan
        //Colin is hot
        double rx = gamepad1.right_stick_x + gamepad2.right_stick_x; //Change to Gmpd2 later
        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        // Rotate the movement direction counter to the bot's rotation
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double frontLeftPower = speedFactor * (rotY + rotX + rx) / denominator;
        double backLeftPower = speedFactor * (rotY - rotX + rx) / denominator;
        double frontRightPower = speedFactor * (rotY - rotX - rx) / denominator;
        double backRightPower = speedFactor * (rotY + rotX - rx) / denominator;
        drive.setMotorPowers(frontLeftPower, backLeftPower, backRightPower, frontRightPower);
        telemetry.addData("thing that Dylan wants: ", speedFactor);
        telemetry.update();
    }
}
//