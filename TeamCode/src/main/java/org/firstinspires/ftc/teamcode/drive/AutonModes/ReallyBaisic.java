package org.firstinspires.ftc.teamcode.drive.AutonModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous
public class ReallyBaisic extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        waitForStart();

        if(isStopRequested()) return;
        while(opModeIsActive()){
            drive.setMotorPowers(0.3,-0.3,0.3,-0.3);
        }

    }
}
