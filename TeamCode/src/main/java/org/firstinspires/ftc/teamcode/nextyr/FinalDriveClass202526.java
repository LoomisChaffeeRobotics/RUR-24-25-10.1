package org.firstinspires.ftc.teamcode.nextyr;

import android.util.Size;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

public class FinalDriveClass202526 extends OpMode {
    Pose3D cameraPos;
    IMU imu;
    SampleMecanumDrive drive;
    VisionPortal myVisionPortal;
    AprilTagProcessor myAprilTagProcessor;
    private Position cameraPosition = new Position(DistanceUnit.INCH,
            5, -5, 5.5, 0);
    private YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES,
            180, -90, 0, 0);

    @Override
    public void init() {
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.initialize(parameters);
        drive = new SampleMecanumDrive(hardwareMap);
        myAprilTagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setDrawCubeProjection(true)
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .setLensIntrinsics(996.968, 996.968, 929.592, 530.676)
                .setCameraPose(cameraPosition, cameraOrientation)
                .build();
        VisionPortal.Builder myVisionPortalBuilder = new VisionPortal.Builder();
        myVisionPortalBuilder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        myVisionPortalBuilder.addProcessor(myAprilTagProcessor);
        myVisionPortalBuilder.setCameraResolution(new Size(1920, 1080));
        myVisionPortalBuilder.setStreamFormat(VisionPortal.StreamFormat.YUY2);
        myVisionPortalBuilder.enableLiveView(true);
        myVisionPortalBuilder.setAutoStopLiveView(true);
        myVisionPortal = myVisionPortalBuilder.build();
    }
    public void loop() {
        double speedFactor = Math.min(1.3 - gamepad2.left_trigger, 1); // so driver can slow down

        double y = -gamepad2.left_stick_y; // Remember, Y stick value is reversed
        double x = gamepad2.left_stick_x; //dylan is my bf // If your going to write my name, bother to capitalize it. - Dylan // It's "you're" not "your" - Colin
        double rx = gamepad1.right_stick_x *.3 + gamepad2.right_stick_x;
        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS); // Rotate the movement direction counter to the bot's rotation
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        rotX = rotX * 1.096024278; //counteract for imperfect strafing
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double frontLeftPower = speedFactor * (rotY + rotX + rx) / denominator;
        double backLeftPower = speedFactor * (rotY - rotX + rx) / denominator;
        double frontRightPower = speedFactor * (rotY - rotX - rx) / denominator;
        double backRightPower = speedFactor * (rotY + rotX - rx) / denominator;
        drive.setMotorPowers(frontLeftPower, backLeftPower, backRightPower, frontRightPower);
        telemetry.addData("x:", x);
        telemetry.update();
    }
    public double cameraPoseUpdate() {
        List<AprilTagDetection> currentDetections = myAprilTagProcessor.getDetections();
        Pose2d poseRobot = new Pose2d(0, 0, 0);
        double headingNew = 5;
        if (!currentDetections.isEmpty()) {
            AprilTagDetection aprilTag1 = currentDetections.get(0);
            cameraPos = aprilTag1.robotPose; // hopefully in inches
            double heading = Math.toRadians(cameraPos.getOrientation().getYaw() + 90);
            telemetry.addData("heading ", Math.toDegrees(heading));
            telemetry.addData("initial heading ", cameraPos.getOrientation().getYaw());
            telemetry.addData("aprilX ", aprilTag1.metadata.fieldPosition.getData()[0]);
            telemetry.addData("aprilY ", aprilTag1.metadata.fieldPosition.getData()[1]);
            double x = cameraPos.getPosition().x;
            double y = cameraPos.getPosition().y;
            headingNew =  cameraPos.getOrientation().getYaw();
            poseRobot = new Pose2d(
                    x, y, heading
            );
            drive.setPoseEstimate(poseRobot);
            telemetry.update();
        }
        telemetry.addData("poseRobot: ", poseRobot);
        return(headingNew);
    }
}
