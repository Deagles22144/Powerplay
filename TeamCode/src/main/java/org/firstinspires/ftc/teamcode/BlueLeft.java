package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous (name="BlueLeft", group="AutoLeft")
public class BlueLeft extends QRcode {


    @Override
    public void runOpMode() {
        super.runOpMode();

        Pose2d startPose = new Pose2d(35, 65, Math.toRadians(90));
        drive.setPoseEstimate(startPose);

        while (!isStarted() && !isStopRequested()) {
            QrScan();

        }

        TrajectorySequence Preload = drive.trajectorySequenceBuilder(startPose)
                .setTangent(Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(35, 25), Math.toRadians(-90))
                .addDisplacementMarker(5,() -> {
                    elevatorHigh();
                    tilt.setPosition(tiltHigh);
                })
                .splineToSplineHeading(new Pose2d(32, 10, Math.toRadians(45)), Math.toRadians(-110))
                .waitSeconds(1)
                .build();

        TrajectorySequence FirstDriveToCones = drive.trajectorySequenceBuilder(Preload.end())
                .setTangent(Math.toRadians(60))
                .splineToSplineHeading(new Pose2d(43, 15, Math.toRadians(45)),Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(60,15, Math.toRadians(0)),Math.toRadians(0))
                .addDisplacementMarker(8,() -> {
                    elevatorAuto(cones[0]);
                    tilt.setPosition(tiltGround);
                })
                .forward(7.0, SampleMecanumDrive.getVelocityConstraint(5, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        TrajectorySequence ConeUnload = drive.trajectorySequenceBuilder(new Pose2d(-63,15, Math.toRadians(180)))
                .setTangent(Math.toRadians(180))
                .splineToSplineHeading(new Pose2d(45, 15, Math.toRadians(0)),Math.toRadians(180))
                .addDisplacementMarker(5,() -> {
                    elevatorHigh();
                    tilt.setPosition(tiltHigh);
                })
                .splineToSplineHeading(new Pose2d(33, 9 , Math.toRadians(45)),Math.toRadians(-110),SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .waitSeconds(1)
                .build();

        TrajectorySequence ConeLoad = drive.trajectorySequenceBuilder(ConeUnload.end())//FirstBackDrive.end())
                .setTangent(Math.toRadians(60))
                .splineToSplineHeading(new Pose2d(43, 15, Math.toRadians(45)),Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(60,15, Math.toRadians(0)),Math.toRadians(0))
//                .forward(7.0, SampleMecanumDrive.getVelocityConstraint(5, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        TrajectorySequence forwardDrive = drive.trajectorySequenceBuilder(ConeLoad.end())
                .forward(7.0, SampleMecanumDrive.getVelocityConstraint(5, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        /** --------- Park Auto Trajectories ----------**/

        TrajectorySequence ParkMid  = drive.trajectorySequenceBuilder(ConeUnload.end())
                .setTangent(120)
                .lineToLinearHeading(new Pose2d(35,24,Math.toRadians(90)))
                .build();

        TrajectorySequence ParkRight = drive.trajectorySequenceBuilder(ConeUnload.end())
                .setTangent(90)
                .splineToSplineHeading(new Pose2d(35,25,Math.toRadians(115)),Math.toRadians(115))
                .splineToSplineHeading( new Pose2d(64,35,Math.toRadians(180)),Math.toRadians(180))
                .build();

        TrajectorySequence ParkLeft = drive.trajectorySequenceBuilder(ConeUnload.end())
                .setTangent(45)
                .splineToSplineHeading(new Pose2d(28,13,Math.toRadians(180)),Math.toRadians(0))
                .splineToSplineHeading( new Pose2d(10,25,Math.toRadians(-90)),Math.toRadians(90))
                .build();




        waitForStart();


        armPos(0.23);

        drive.followTrajectorySequence(Preload);

        if (elevator1.getCurrentPosition() >= elevatoeHighPos - (elevatoeHighPos / 20) || elevator0.getCurrentPosition() >= elevatoeHighPos - (elevatoeHighPos / 20))
        {
            claw.setPosition(clawOpen);
            sleep(750);
            isGround = true;
            elevatorGround();
        }
        /*if (elevator0.getCurrentPosition() <= elevatorMiddlePos/2 && isGround) {
            armPos(armGround);
            tilt.setPosition(tiltGround);
            isGround = false;
        }

         */
        while (elevator0.getCurrentPosition() >= 250)
        {
            armPos(armLow);
        }

        armPos(armGround);
        tilt.setPosition(tiltGround);
        sleep(600);

        drive.followTrajectorySequence(FirstDriveToCones);


        for (int i = 1; i < 3; i++)
        {

            drive.setPoseEstimate(new Pose2d(-63,15, Math.toRadians(180)));

            claw.setPosition(clawClose);
            tilt.setPosition(tiltGround);
            sleep(1000);
            elevatorAfterColloctAuto();

            drive.followTrajectorySequence(ConeUnload);

/*
            elevatorHigh();
            tilt.setPosition(tiltAuto);

            sleep(2500);
            */

            // drive.followTrajectorySequence(SecondBackDrive);
            //drive.followTrajectorySequence(SecondBackDrive);

            telemetry.addLine("elevator pos: " + elevator1.getCurrentPosition());
            if (elevator1.getCurrentPosition() >= elevatoeHighPos - (elevatoeHighPos / 20))
            {
                claw.setPosition(clawOpen);
                sleep(750);
                isGround = true;
            }

            //elevatorAuto(cones[i]);
            elevatorGround();
            while (elevator0.getCurrentPosition() >= 250)
            {
                armPos(armLow);
            }

            armPos(armGround);
            tilt.setPosition(tiltGround);
            sleep(600);

            if(i < 2)
            {
                drive.followTrajectorySequence(ConeLoad);
                elevatorAuto(cones[i]);
                drive.followTrajectorySequence(forwardDrive);
            }
        }

        elevatorGround();

        if (tagOfInterest == null || tagOfInterest.id == parkLeft)
        {
            //left code
            drive.followTrajectorySequence(ParkLeft);

        } else if (tagOfInterest.id == parkMiddle)
        {
            //middle code
            drive.followTrajectorySequence(ParkMid);

        } else if (tagOfInterest.id == parkRight)
        {
            //right code
            drive.followTrajectorySequence(ParkRight);

        }
    }


}


