/*
package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous (name="BlueRight5Cones", group="AutoLeft")
public class BlueRight5Cones extends QRcode {


    @Override
    public void runOpMode() {
        super.runOpMode();

        Pose2d startPose = new Pose2d(-36, 65, Math.toRadians(-90));
        drive.setPoseEstimate(startPose);

        while (!isStarted() && !isStopRequested()) {
            QrScan();
        }


        TrajectorySequence Preload = drive.trajectorySequenceBuilder(startPose)
                .setTangent(Math.toRadians(-90))
                .splineToSplineHeading(new Pose2d(-37 , 4, Math.toRadians(-90)), Math.toRadians(-90))
                */
/*.addDisplacementMarker(40, () -> {
                    armPos(0.6);
                })*//*

                .addDisplacementMarker(() -> {
                    elevatorMid();
                    armPos(armMid);
                })
                .lineToSplineHeading(new Pose2d(-33, 15, Math.toRadians(-130)))
                .build();

        TrajectorySequence coneLoad = drive.trajectorySequenceBuilder(Preload.end())
                .setTangent(Math.toRadians(-135))
                //.lineToLinearHeading((new Pose2d(58, 12, Math.toRadians(0))))
                .splineToSplineHeading(new Pose2d(-40, 13, Math.toRadians(180)), Math.toRadians(180))
                .splineToSplineHeading(new Pose2d(-66, 13, Math.toRadians(180)), Math.toRadians(180))
                .build();


        TrajectorySequence coneUnload = drive.trajectorySequenceBuilder(new Pose2d(-63,13, Math.toRadians(180))*/
/*coneLoad.end()*//*
)
                .setTangent(Math.toRadians(0))
                .addDisplacementMarker( () -> {
                    elevatorMid();
                    armPos(armMid);
                })
                .splineToSplineHeading(new Pose2d(-44,13,Math.toRadians(180)), Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(-33, 13, Math.toRadians(-130)), Math.toRadians(35))
                // .lineToLinearHeading(new Pose2d(58 , 15 ,Math.toRadians(-45)))
                .waitSeconds(0.2)

                */
/*.setTangent(45)
                .lineToLinearHeading(new Pose2d(58, 15, Math.toRadians(-45)))*//*

                .build();

        */
/** --------- Park Auto Trajectories ----------**//*


        TrajectorySequence ParkMid  = drive.trajectorySequenceBuilder(coneUnload.end())
                .setTangent(180)
                .lineToLinearHeading(new Pose2d(-36,25  ,Math.toRadians(-90)))
                .build();

        TrajectorySequence ParkRight = drive.trajectorySequenceBuilder(coneUnload.end())
                .setTangent(Math.toRadians(180))
//                .splineToSplineHeading(new Pose2d(-35, 30, Math.toRadians(180)),Math.toRadians(80 ))
//                .splineToConstantHeading(new Vector2d(-65, 35),Math.toRadians(180))
                .lineToLinearHeading(new Pose2d(-65,12,Math.toRadians(180)))
                .build();

        TrajectorySequence ParkLeft = drive.trajectorySequenceBuilder(coneUnload.end())
                .setTangent(-50)
                //.splineToSplineHeading(new Pose2d(-26,12,Math.toRadians(0)),Math.toRadians(0))
                //.splineToSplineHeading( new Pose2d(-12,25,Math.toRadians(90)),Math.toRadians(90))
                .lineToLinearHeading(new Pose2d(-10,15,Math.toRadians(-90)))
                .build();


        waitForStart();

        armPos(armLoadCone);
        drive.followTrajectorySequence(Preload);
        //armPos(armHigh);
        sleep(450);
        armPos(armPreRelease);
//        sleep(300);
        claw.setPosition(clawOpen);
        sleep(250);
        claw.setPosition(clawClose);
        sleep(100);
        elevatorGround();
        sleep(300);

        for (int i = 0; i < 4; i++) {

            elevatorAuto(cones[i]);
            claw.setPosition(clawOpen);
            drive.followTrajectorySequence(coneLoad);
            claw.setPosition(clawClose);
            drive.setPoseEstimate(new Pose2d(-63, drive.getPoseEstimate().getY(), drive.getPoseEstimate().getHeading()));
            sleep(100);
            elevatorTargetPosition(cones[i] + 350);
            sleep(200);
            drive.followTrajectorySequence(coneUnload);
            // armPos(armHigh);
            sleep(200);
//            armPos(armMid);
            armPos(armPreRelease);
//            sleep(300);
            claw.setPosition(clawOpen);
            sleep(300);
            claw.setPosition(clawClose);
            sleep(200);
            elevatorGround();
            sleep(300);
        }

        if (tagOfInterest == null || tagOfInterest.id == parkLeft)
        {
            //left code
            armPos(armGround);
            drive.followTrajectorySequence(ParkLeft);

        } else if (tagOfInterest.id == parkMiddle)
        {
            //middle code
            armPos(armGround);
            drive.followTrajectorySequence(ParkMid);

        } else if (tagOfInterest.id == parkRight)
        {
            //right code
            armPos(armGround);
            drive.followTrajectorySequence(ParkRight);

        }






    }
}*/
