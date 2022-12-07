package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous (name="RedAUTOcar", group="Robot")
public class RedAUTOcar extends QRcode{


    @Override
    public void runOpMode() {
        super.runOpMode();

        while (!isStarted() && !isStopRequested()){
            QrScan();

        }
        waitForStart();
        Trajectory firstdrop = drive.trajectoryBuilder(new Pose2d(-36.4,-66.8, Math.toRadians(79)), Math.toRadians(79))
                .splineToSplineHeading(new Pose2d(-38.4,-12.4, Math.toRadians(-180)), Math.toRadians(-138))
                .build();
        drive.followTrajectory(firstdrop);
        elevatorHigh();
        tilt.setPosition(0.2);
        sleep(200);
        claw.setPosition(clawOpen);
        sleep(500);
        elevatorGround();
        for (int i = 0; i<3; i++) {
            Trajectory getconus = drive.trajectoryBuilder(new Pose2d(-52.4, -12, Math.toRadians(-138)), Math.toRadians(-180))
                    .splineToSplineHeading(new Pose2d(-38.8, -10.4, Math.toRadians(-180)), Math.toRadians(-138))
                    .build();
            drive.followTrajectory(getconus);
            claw.setPosition(clawClose);
            Trajectory drop = drive.trajectoryBuilder(new Pose2d(-40, -11.2, Math.toRadians(-180)), Math.toRadians(-138))
                    .build();
            drive.followTrajectory(drop);
            elevatorHigh();
            tilt.setPosition(0.2);
            sleep(200);
            claw.setPosition(clawOpen);
            sleep(500);
            elevatorGround();
        }
        if (tagOfInterest == null || tagOfInterest.id == parkLeft) {
            //left code

        } else if (tagOfInterest.id == parkMiddle) {
            //middle code

        }
        else if (tagOfInterest.id == parkRight) {
            //right code

        }
    }
}
