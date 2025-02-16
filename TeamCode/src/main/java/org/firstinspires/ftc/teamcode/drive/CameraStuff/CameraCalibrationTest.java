package org.firstinspires.ftc.teamcode.drive.CameraStuff;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
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
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;


import java.util.List;

@TeleOp
public class CameraCalibrationTest extends OpMode {
    private Position cameraPosition = new Position(DistanceUnit.INCH,
            5, -5, 5.5, 0);
    private YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES,
            180, -90, 0, 0);
    IMU imu;
    VisionPortal myVisionPortal;
    AprilTagProcessor myAprilTagProcessor;
    SampleMecanumDrive drive;
    FtcDashboard dash;
    Telemetry telemetry;

    @Override
    public void init() {
        drive = new SampleMecanumDrive(hardwareMap);
        dash = FtcDashboard.getInstance();
        telemetry = dash.getTelemetry();
        imu = hardwareMap.get(IMU.class, "imu");
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
        drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void loop() {
        cameraPoseUpdate();

    }

    public Pose2d cameraPoseUpdate() {
        List<AprilTagDetection> currentDetections = myAprilTagProcessor.getDetections();
        Pose2d poseRobot = new Pose2d(0,0,0);
        if (!currentDetections.isEmpty()) {
            AprilTagDetection aprilTag1 = currentDetections.get(0);
            Pose3D cameraPos = aprilTag1.robotPose; // hopefully in inches
            double heading = Math.toRadians(cameraPos.getOrientation().getYaw()+90);
            telemetry.addData("heading ", Math.toDegrees(heading));
            telemetry.addData("initial heading ", cameraPos.getOrientation().getYaw());
            telemetry.addData("aprilX ", aprilTag1.metadata.fieldPosition.getData()[0]);
            telemetry.addData("aprilY ", aprilTag1.metadata.fieldPosition.getData()[1]);
            double x = cameraPos.getPosition().x;
            double y = cameraPos.getPosition().y;
            poseRobot = new Pose2d(
                    x, y, heading
            );
            drive.setPoseEstimate(poseRobot);
            telemetry.update();
        }
        telemetry.addData("poseRobot: ", poseRobot);
        return poseRobot;
    }

}
