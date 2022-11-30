package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous (name="autoCar", group="Robot")
public class autoCar extends QRcode{


    @Override
    public void runOpMode() {
        super.runOpMode();



        while (!isStarted() && !isStopRequested()){
            QrScan();

        }
        waitForStart();

        Trajectory traj = drive.trajectoryBuilder(new Pose2d(-36.8,69.6, Math.toRadians(140)), Math.toRadians(-92))
                .splineToSplineHeading(new Pose2d(-38.8,12, Math.toRadians(-180)), Math.toRadians(140))
                .build();

        Trajectory traj1 = drive.trajectoryBuilder(new Pose2d(-40,11.6, Math.toRadians(-180)), Math.toRadians(143))
                .splineToSplineHeading(new Pose2d(-52,12, Math.toRadians(-180)), Math.toRadians(-180))
                .build();

        Trajectory traj2 = drive.trajectoryBuilder(new Pose2d(-52.8,12, Math.toRadians(-180)), Math.toRadians(-180))
                .splineToSplineHeading(new Pose2d(-43.6,12.4, Math.toRadians(-180)), Math.toRadians(143))
                .build();



        drive.followTrajectory(traj);
        elevatorHigh();
        tilt.setPosition(0.2);
        sleep(200);
        claw.setPosition(clawOpen);
        sleep(500);
        elevatorGround();

        for (int i = 0; i<3; i++) {
            drive.followTrajectory(traj1);
            claw.setPosition(clawClose);
            drive.followTrajectory(traj2);
            elevatorHigh();
            tilt.setPosition(0.2);
            sleep(200);
            claw.setPosition(clawOpen);
            sleep(500);
            elevatorGround();
        }


        if (tagOfInterest == null || tagOfInterest.id == left) {
            //left code

        } else if (tagOfInterest.id == middle) {
            //middle code
                    }
        else if (tagOfInterest.id == right) {
            //right code

        }
    }


}
