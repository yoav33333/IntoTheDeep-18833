package org.firstinspires.ftc.teamcode.opModes.auto;

import static org.firstinspires.ftc.teamcode.commands.extendoCommand.getExtendoOpenCommand;
import static org.firstinspires.ftc.teamcode.subsystems.deposit.getCatchPixel;
import static org.firstinspires.ftc.teamcode.subsystems.deposit.getPostIntakeState;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathBuilder;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.commands.extendoCommand;
import org.firstinspires.ftc.teamcode.opModes.AutoBaseJava;
import org.firstinspires.ftc.teamcode.subsystems.armClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.clawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.deposit;
import org.firstinspires.ftc.teamcode.subsystems.linearSlides;
import org.firstinspires.ftc.teamcode.util.RunNonBlocking;
import org.firstinspires.ftc.teamcode.util.WaitUntil;

import dev.frozenmilk.mercurial.commands.groups.Parallel;
import dev.frozenmilk.mercurial.commands.groups.Sequential;
import dev.frozenmilk.mercurial.commands.util.Wait;

@Config
@Autonomous
public class java5specNonSpin extends AutoBaseJava {
    public java5specNonSpin() {super(Side.chamber);}

    public Pose startPose = this.startingPoseChamber;
    public Pose chamberPose1 = new Pose(-34, -5, Math.toRadians(180));
    public Pose chamberPose2 = new Pose(-33, -7, Math.toRadians(180));
    public Pose chamberPose3 = new Pose(-32.8, -2, Math.toRadians(180));
    public Pose chamberPose4 = new Pose(-33.5, -4, Math.toRadians(180));
    public Pose chamberPose5 = new Pose(-33.5, -6, Math.toRadians(180));
    public Pose dragPose1 = new Pose(-45.2, -27, Math.toRadians(-46));
    public Pose dragPose2 = new Pose(-44.3, -37.2, Math.toRadians(-46));
    public Pose dragPose3 = new Pose(-38, -42, Math.toRadians(-40));
    public Pose dragPoseTurn1 = new Pose(-45.2, -28, Math.toRadians(-45));
    public Pose dragPoseTurn2 = new Pose(-44.3, -38.2, Math.toRadians(-45));
    public Pose dragPoseTurn3 = new Pose(-45, -43, Math.toRadians(-45-90));
    public Pose pickup1Pose = new Pose(-57, -38, Math.toRadians(-135));
    public Pose pickup12Pose = new Pose(-52.3, -34, Math.toRadians(-135));
    public Pose pickup13Pose = new Pose(-62, -38, Math.toRadians(0));
    public Pose pickup2Pose = new Pose(-61.3, -25, Math.toRadians(180+80));
    public Pose parkPose = new Pose(-47, -22, Math.toRadians(180+40));
    public static PathBuilder builder = new PathBuilder();

    public static PathChain line1 = builder
            .addPath(
                    new BezierLine(
                            new Point(6.790, 59.707, Point.CARTESIAN),
                            new Point(38.634, 70.244, Point.CARTESIAN)
                    )
            )
            .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(180))
            .build();

    public static PathChain line2 = builder
            .addPath(
                    new BezierCurve(
                            new Point(38.634, 70.244, Point.CARTESIAN),
                            new Point(1.873170731707317, 30.907317073170727, Point.CARTESIAN),
                            new Point(56.898, 40.507, Point.CARTESIAN),
                            new Point(55.024, 22.946, Point.CARTESIAN)
                    )
            )
            .setConstantHeadingInterpolation(Math.toRadians(180))
            .build();

    public static PathChain line3 = builder
            .addPath(
                    new BezierCurve(
                            new Point(55.024, 22.946, Point.CARTESIAN),
                            new Point(11.005, 22.010, Point.CARTESIAN),
                            new Point(75.161, 29.502, Point.CARTESIAN),
                            new Point(51.980, 11.005, Point.CARTESIAN)
                    )
            )
            .setConstantHeadingInterpolation(Math.toRadians(180))
            .build();

    public static PathChain line4 = builder
            .addPath(
                    new BezierCurve(
                            new Point(51.980, 11.005, Point.CARTESIAN),
                            new Point(12.410, 13.815, Point.CARTESIAN),
                            new Point(62.049, 15.220, Point.CARTESIAN),
                            new Point(57.834, 6.790, Point.CARTESIAN)
                    )
            )
            .setConstantHeadingInterpolation(Math.toRadians(180))
            .build();

    public static PathChain line5 = builder
            .addPath(
                    new BezierLine(
                            new Point(57.834, 6.790, Point.CARTESIAN),
                            new Point(28.566, 7.493, Point.CARTESIAN)
                    )
            )
            .setConstantHeadingInterpolation(Math.toRadians(180))
            .build();

    public static PathChain line6 = builder
            .addPath(
                    new BezierLine(
                            new Point(28.566, 7.493, Point.CARTESIAN),
                            new Point(37.698, 66.498, Point.CARTESIAN)
                    )
            )
            .setConstantHeadingInterpolation(Math.toRadians(180))
            .build();

    public static PathChain line7 = builder
            .addPath(
                    new BezierCurve(
                            new Point(37.698, 66.498, Point.CARTESIAN),
                            new Point(20.605, 64.859, Point.CARTESIAN),
                            new Point(12.000, 40.000, Point.CARTESIAN)
                    )
            )
            .setTangentHeadingInterpolation()
            .build();
    public static PathChain line8 = builder
            .addPath(
                    new BezierCurve(
                            new Point(12.000, 40.000, Point.CARTESIAN),
                            new Point(20.605, 64.859, Point.CARTESIAN),
                            new Point(37.698, 66.498, Point.CARTESIAN)
                    )
            )
            .setTangentHeadingInterpolation()
            .setReversed(true)
            .build();
    static PathChain scorePreload;
    static PathChain getToDrag1;
    static PathChain turnDrag1;
    static PathChain getToDrag2;
    static PathChain turnDrag2;
    static PathChain getToDrag3;
    static PathChain turnDrag3;
    static PathChain specialHPIntake;
    static PathChain specialHPIntake1;
    static PathChain specialHPIntake1score;
    static PathChain specialHPIntake2;
    static PathChain specialHPIntake2score;
    static PathChain dragToHP;
    static PathChain scorePickup1;
    static PathChain scorePickup2;
    static PathChain scorePickup3;
    static PathChain scorePickup4;
    static PathChain pickup1;
    static PathChain pickup2;
    static PathChain pickup3;
    static PathChain pickup4;
    static PathChain park;
    @Override
    public void myInit() {

        scorePreload = makeLinePath(startPose, chamberPose1);
        getToDrag1 = makeLinePath(chamberPose1, dragPose1);
        turnDrag1 = makeLinePath(dragPose1, dragPoseTurn1);
        getToDrag2 = makeLinePath(dragPoseTurn1, dragPose2);
        turnDrag2 = makeLinePath(dragPose2, dragPoseTurn2);
        getToDrag3 = makeLinePath(dragPoseTurn2, dragPose3);
        turnDrag3 = makeLinePath(dragPose3, dragPoseTurn3);
//        specialHPIntake = makeLinePath(dragPoseTurn3, pickup1Pose);
        specialHPIntake = makeLinePath(dragPose2, pickup1Pose);
        specialHPIntake1 = makeLinePath(chamberPose2, pickup12Pose);
        park = makeLinePath(chamberPose2, parkPose);
        specialHPIntake1score = makeLinePath(pickup12Pose, chamberPose2);
//        dragToHP = makeCurvePath(chamberPose1, dragPose1, dragPoseTurn1, dragPose2, dragPoseTurn2, dragPose3, dragPoseTurn3,pickup1Pose);
        scorePickup1 = makeLinePath(pickup1Pose, chamberPose2);
        scorePickup2 = makeLinePath(pickup2Pose, chamberPose3);
        scorePickup3 = makeLinePath(pickup2Pose, chamberPose4);
        scorePickup4 = makeLinePath(pickup2Pose, chamberPose5);
        pickup1 = makeLinePath(chamberPose2, pickup2Pose);
        pickup2 = makeLinePath(chamberPose3, pickup2Pose);
        pickup3 = makeLinePath(chamberPose4, pickup2Pose);
        pickup4 = makeLinePath(chamberPose5, pickup2Pose);
//        park = makeLinePath(basketScore, parkPose);
//        clawSubsystem.getClawRotationServo().setPosition(0.5);
        deposit.armOutHalf();
        deposit.closeClawRaw();
        clawSubsystem.openClaw();
    }

    @Override
    public void myStart() {
//        deposit.armOut();
        linearSlides.getRunToPosition().schedule();
        new Sequential(
            extendoCommand.getExtendoReset(),
            linearSlides.getGoToHighChamberUp(),
            followPath(scorePreload).with(
                new Sequential(
                    new WaitUntil(()->linearSlides.getPose()>500),
                    deposit.getArmOut()
                )
            ),
            deposit.getSlamArmDown(),
            new Parallel(
                new Sequential(
                    new WaitUntil(()->follower.getCurrentTValue()>0.2),
                    extendoCommand.getExtendoOpenCommandAutoPush(),
                    armClawSubsystem.getArmUp(),
                    clawSubsystem.getTurnLeft()
                ),
                new RunNonBlocking(new Sequential(
                        new WaitUntil(()->follower.getCurrentTValue()>0.9),
                        armClawSubsystem.getExtendoPushState()
                        )),
//                instantCommand(()-> FollowerConstants.headingPIDFCoefficients.P = 0.8),
                followPath(getToDrag1)
            ),
//            new Wait(0.25),
//            clawSubsystem.getCloseClaw(),
//            new Wait(0.1),
//            instantCommand(()-> FollowerConstants.headingPIDFCoefficients.P = 1.9),
                turn(90).with(new Wait(0.8)),
                armClawSubsystem.getArmUp(),

//            clawSubsystem.getOpenClaw(),
//            instantCommand(()-> FollowerConstants.headingPIDFCoefficients.P = 0.75),
                new RunNonBlocking(new Sequential(
                        new WaitUntil(()->follower.getCurrentTValue()>0.9),
                        armClawSubsystem.getExtendoPushState()
                )),
                followPath(getToDrag2),

                turn(90).with(new Wait(0.8)),
//            new Wait(0.25),
//            clawSubsystem.getCloseClaw(),
//            new Wait(0.1),
//            instantCommand(()-> FollowerConstants.headingPIDFCoefficients.P = 1.9),
//                followPath(turnDrag2),
                armClawSubsystem.getArmUp(),
                new RunNonBlocking(new Sequential(
                        new WaitUntil(()->follower.getCurrentTValue()>0.9),
                        armClawSubsystem.getExtendoPushState()
                )),
                followPath(getToDrag3),
//            new Wait(0.25),
//            clawSubsystem.getCloseClaw(),
//            new Wait(0.1),
//            instantCommand(()-> FollowerConstants.headingPIDFCoefficients.P = 1.9),
            followPath(turnDrag3),
//            clawSubsystem.getOpenClaw(),
            extendoCommand.getExtendoCloseCommandSimple(),

            new Parallel(
                new Sequential(
                    new Wait(0.2),
                    linearSlides.getGoToLowChamberNoRC()
                ),
                followPath(specialHPIntake),
                new Sequential(
                    deposit.getIntakeCommand(),
                    new Wait(0.1),
                    deposit.getRelease(),
                    new WaitUntil(()-> !follower.isBusy()),
                    getCatchPixel().raceWith(slowX),
                    new RunNonBlocking(
                            new Sequential(new Wait(0.2),
                    getPostIntakeState())
                ))
            ),
            linearSlides.getGoToHighChamberUp(),
            followPath(scorePickup1),
            deposit.getSlamArmDown(),
                new Parallel(
                    new Sequential(
                        new Wait(0.2),
                        linearSlides.getGoToLowChamberNoRC()
                    ),
                    followPath(specialHPIntake1),
                    new Sequential(
                        deposit.getIntakeCommand(),
                        new Wait(0.1),
                        deposit.getRelease(),
                        new WaitUntil(()-> !follower.isBusy()),
                        getCatchPixel().raceWith(slowX),
                            new RunNonBlocking(
//                                    new Sequential(new Wait(0.2),
                            getPostIntakeState())
                    )
            ),
            linearSlides.getGoToHighChamberUp(),
            followPath(specialHPIntake1score),
            deposit.getSlamArmDown(),
                new Parallel(
                new Sequential(
                    new Wait(0.2),
                    linearSlides.getGoToLowChamberNoRC()
                ),
                followPath(specialHPIntake1),
                new Sequential(
                    deposit.getIntakeCommand(),
                    new Wait(0.1),
                    deposit.getRelease(),
                    new WaitUntil(()-> !follower.isBusy()),
                    getCatchPixel().raceWith(slowX),
                        new RunNonBlocking(
//                                new Sequential(new Wait(0.2),
                                        getPostIntakeState())
                )
            ),
            linearSlides.getGoToHighChamberUp(),
            followPath(specialHPIntake1score),
            deposit.getSlamArmDown(),
            new Parallel(
                new Sequential(
                    new Wait(0.2),
                    linearSlides.getGoToLowChamberNoRC()
                ),
                followPath(specialHPIntake1),
                new Sequential(
                    deposit.getIntakeCommand(),
                    new Wait(0.1),
                    deposit.getRelease(),
                    new WaitUntil(()-> !follower.isBusy()),
                    getCatchPixel().raceWith(slowX),
                        new RunNonBlocking(
//                                new Sequential(new Wait(0.2),
                                        getPostIntakeState())
                )
            ),
            linearSlides.getGoToHighChamberUp(),
            followPath(specialHPIntake1score),
            deposit.getSlamArmDown(),
                new Parallel(
//                    new Sequential(
//                            new Wait(0.2),
//                        linearSlides.getGoToLowChamberNoRC()
//                    ),
                    new Wait(0.4).then(getExtendoOpenCommand()),
                    followPath(park)

                ),
//            new Parallel(
//                new Sequential(
//                    new Wait(0.2),
//                    linearSlides.getGoToLowChamberNoRC()
//                ),
//                followPath(specialHPIntake1),
//                new Sequential(
//                    deposit.getIntakeCommand(),
//                    new Wait(0.5),
//                    deposit.getRelease(),
//                    getCatchPixel(),
//                    new Wait(0.3),
//                    getPostIntakeState()
//                )
//            ),
//            linearSlides.getGoToHighChamber(),
//            followPath(specialHPIntake1score),
//            deposit.getSlamSeq(),
//                new Parallel(
//                new Sequential(
//                    new Wait(0.2),
//                    linearSlides.getGoToLowChamberNoRC()
//                ),
//                followPath(specialHPIntake1),
//                new Sequential(
//                    deposit.getIntakeCommand(),
//                    new Wait(0.5),
//                    deposit.getRelease(),
//                    getCatchPixel(),
//                    new Wait(0.3),
//                    getPostIntakeState()
//                )
//            ),
//            linearSlides.getGoToHighChamber(),
//            followPath(specialHPIntake1score),
//            deposit.getSlamSeq(),
//            new Parallel(
//                followPath(pickup1),
//                extendoCommand.getExtendoOpenCommandAuto()
//            ),
//            clawSubsystem.getCloseClaw(),
//            new Wait(0.1),
//            new Parallel(
//                followPath(scorePickup2),
//                linearSlides.getGoToHighChamber(),
//                getExtendoCloseCommandAuto()
//            ),
//            new WaitUntil(()->linearSlides.getPose()>1130),
//            deposit.getSlamSeq(),
//
//            new Wait(0.2),
//                new Parallel(
//                        followPath(pickup2),
//                        extendoCommand.getExtendoOpenCommandAuto()
//                ),
//                clawSubsystem.getCloseClaw(),
//                new Wait(0.1),
//                new Parallel(
//                        followPath(scorePickup3),
//                        linearSlides.getGoToHighChamber(),
//                        getExtendoCloseCommandAuto()
//                ),
//                new WaitUntil(()->linearSlides.getPose()>1130),
//                deposit.getSlamSeq(),
//                new Wait(0.2),
////            followPath(pickup2),
////            followPath(scorePickup3),
//                new Parallel(
//                        followPath(pickup3),
//                        extendoCommand.getExtendoOpenCommandAuto()
//                ),
//                clawSubsystem.getCloseClaw(),
//                new Wait(0.1),
//                new Parallel(
//                        followPath(scorePickup4),
//                        linearSlides.getGoToHighChamber(),
//                        getExtendoCloseCommandAuto()
//                ),
//                new WaitUntil(()->linearSlides.getPose()>1130),
//                deposit.getSlamSeq(),
//                new Wait(0.2),
//                followPath(pickup4),
//                followPath(pickup3),
//                followPath(scorePickup4),
            finishAuto
        );
        new Sequential(
            followPath(line1),
            followPath(line2),
            followPath(line3),
            followPath(line4),
            followPath(line5),
            followPath(line6),
            followPath(line7),
            followPath(line8),
            followPath(line7),
            followPath(line8),
            followPath(line7),
            followPath(line8)
        ).schedule();

    }
    @Override
    public void myStop(){
        linearSlides.setStartingPose(linearSlides.getPose());
    }
}
