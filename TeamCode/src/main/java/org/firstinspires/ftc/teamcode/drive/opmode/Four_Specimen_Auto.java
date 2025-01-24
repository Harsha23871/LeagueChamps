package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(group = "a", name="4 Specimen Auto ")
public class    Four_Specimen_Auto extends LinearOpMode {
    public DcMotor elevator = null;
    public Servo claw,wrist = null;



    public void runOpMode() {
        elevator = hardwareMap.get(DcMotor.class,"elevator_motor");
        claw =  hardwareMap.get(Servo.class, "claw");
        wrist = hardwareMap.get(Servo.class, "wrist");
        elevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


//        hwMapForAuto.elevator_Scoring_Pos();  // Moves elevator to scoring position
//        hwMapForAuto.elevator_Resting_Pos();
        //elevator = hardwareMap.get(DcMotor.class,"elevator_motor");
        claw.setDirection(Servo.Direction.FORWARD);
        claw.setPosition(1);

        waitForStart();

//        elevator.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
//        elevator.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        // Moves elevator to resting position

        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(11,  -61, Math.toRadians(270));
        drive.setPoseEstimate(startPose);
        // to the submersible
        TrajectorySequence trajectory0 = drive.trajectorySequenceBuilder(new Pose2d(11, -61, Math.toRadians(270.00)))
                .splineToConstantHeading(new Vector2d(0, -29), Math.toRadians(34.39),
                        SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .build();
        //to the parking zone

        TrajectorySequence trajectory1 = drive.trajectorySequenceBuilder(new Pose2d(0, -29, Math.toRadians(90.00)))
                .lineTo(new Vector2d(46, -55))
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(5, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))  // Velocity constraint
                .setAccelConstraint(SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))  // Acceleration constraint
                .build();


        TrajectorySequence trajectory1uhoh = drive.trajectorySequenceBuilder(new Pose2d(-0.07, -33.74, Math.toRadians(270)))
                .lineToLinearHeading(new Pose2d(40, -64, Math.toRadians(95)))
                .build();


        TrajectorySequence park = drive.trajectorySequenceBuilder(new Pose2d(-0.07, -33.74, Math.toRadians(270.00)))
                .lineToConstantHeading(new Vector2d(40, -64))
                .build();

        TrajectorySequence trajectory3 = drive.trajectorySequenceBuilder(trajectory1uhoh.end())

                .lineToLinearHeading(new Pose2d(0, -31, Math.toRadians(270.00)))
                .build();

        TrajectorySequence trajectory4 = drive.trajectorySequenceBuilder(trajectory1uhoh.end())
                .lineToLinearHeading(new Pose2d(-7, -31, Math.toRadians(270.00)))
                .build();



        TrajectorySequence Push = drive.trajectorySequenceBuilder(trajectory0.end())
                .lineToConstantHeading(new Vector2d(34, -47.38))
                .lineToConstantHeading(new Vector2d(33.59, -4.37))
                .lineToConstantHeading(new Vector2d(40, -4.37))
                .lineTo(new Vector2d(47, -55))//y = - 64
                .lineTo(new Vector2d(37, -53))//y = - 64
                .lineTo(new Vector2d(37, -64))//y = - 64
                .build();

        TrajectorySequence PushTest = drive.trajectorySequenceBuilder(trajectory0.end())
                .lineTo(new Vector2d(33, -39.39))
                .splineTo(new Vector2d(37.20, -13.03), Math.toRadians(90.63))
                .lineTo(new Vector2d(47.68, -10.26))
                .lineTo(new Vector2d(47.98, -57.15))
                .lineTo(new Vector2d(49.87, -9.39))
                .lineTo(new Vector2d(54.67, -10.12))
                .lineTo(new Vector2d(53.95, -61.23))
                .build();

//                .lineToConstantHeading(new Vector2d(33, -47))
//                .lineToConstantHeading(new Vector2d(42.5, -3))
//                .lineToLinearHeading(new Pose2d(49, -3.5, Math.toRadians(1.25)))
//                .lineTo(new Vector2d(45.0, -52.5))
//                .lineToConstantHeading(new Vector2d(51, -2.50))
//                .splineTo(new Vector2d(59.0, -2.5), Math.toRadians(1.01))
//                .lineToConstantHeading(new Vector2d(55.5, -55))
//                .build();







        wrist.setPosition(0);
        elevator.setTargetPosition(2000);                  //FIRST SPECIMEN
        elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elevator.setPower(0.6);
        drive.followTrajectorySequence(trajectory0);
        elevator.setTargetPosition(1400);
        elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elevator.setPower(0.6);
        sleep(500);
        claw.setPosition(0.7);
        elevator.setTargetPosition(0);
        elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elevator.setPower(0.6);

//     claw position 1 is close
        //0.7 is open
        //RETURN TO WALL AND INTAKE
        drive.followTrajectorySequence(PushTest);

        elevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sleep(500);
        claw.setPosition(1);
        elevator.setTargetPosition(2100);                 //BACK TO SUBMERSIBLE
        elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION); //2nd SPECIMEN SCORED
        elevator.setPower(0.6);
        sleep(500);
        drive.followTrajectorySequence(trajectory3);

        elevator.setTargetPosition(1400);
        elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elevator.setPower(0.6);
        sleep(500);
        claw.setPosition(0.7);
        elevator.setTargetPosition(0);
        elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elevator.setPower(0.6);
        sleep(500);
        drive.followTrajectorySequence(trajectory1uhoh);
        elevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        sleep(500);
        claw.setPosition(1);
        elevator.setTargetPosition(2100);
        elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elevator.setPower(0.6);
        sleep(500);
        drive.followTrajectorySequence(trajectory4);

        elevator.setTargetPosition(1400);
        elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elevator.setPower(0.6);
        sleep(500);
        claw.setPosition(0.7);
        elevator.setTargetPosition(0);
        elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elevator.setPower(0.6);

        drive.followTrajectorySequence(park);





        if (isStopRequested()) return;
        //robot.hwMap();

//        sleep(2000);









        /*TrajectorySequence trajectory7 = drive.trajectorySequenceBuilder(new Pose2d(11, -62, Math.toRadians(90.00)))
                .lineTo(new Vector2d(51.83, -62.36)) //Red side observation Auto( Preload into Observation zone)
                .lineTo(new Vector2d(10.90, -61.03))
                .build();*/








               /*
                TrajectorySequence trajectory7 = drive.trajectorySequenceBuilder(new Pose2d(11, -62, Math.toRadians(90.00)))
                .lineTo(new Vector2d(51.83, -62.36)) //Red side observation Auto( Preload into Observation zone)
                .lineTo(new Vector2d(10.90, -61.03))
                .build();*/










                /*.splineTo(new Vector2d(-49.46, -26.18), Math.toRadians(111.36))



     //   Trajectory myTrajectory = drive.trajectoryBuilder(Traj1.end())
     //           .splineTo(new Vector2d(48 , -48), 0)
      //          .build();
        waitForStart();

        if(isStopRequested()) return;

        drive.followTrajectorySequence(trajectory7);

      /*  Pose2d poseEstimate = drive.getPoseEstimate();
        telemetry.addData("x", poseEstimate.getX());
        telemetry.addData("y", poseEstimate.getY());
        telemetry.addData("heading", poseEstimate.getHeading());
        telemetry.update();*/

        //   drive.followTrajectory(myTrajectory);
//splineTo(new Vector2d(x1, y1), heading)
        //       .splineTo(new Vector2d(x2, y2), heading)
        //       .build();

// -12 -48

       /* SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(12, -62,Math.toRadians(270));
        TrajectorySequence middleSpike = drive.trajectorySequenceBuilder(startPose)
                .lineTo(new Vector2d(12,-62))
//                .turn(Math.toRadians(45)) // Turns 45 degrees counter-clockwise
                .build();


        waitForStart();
        drive.followTrajectorySequence(middleSpike);//might need to change*/


    }   }



