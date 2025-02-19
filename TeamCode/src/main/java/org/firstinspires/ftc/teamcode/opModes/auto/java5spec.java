package org.firstinspires.ftc.teamcode.opModes.auto;

import static org.firstinspires.ftc.teamcode.commands.extendoCommand.getExtendoCloseCommandAuto;
import static org.firstinspires.ftc.teamcode.commands.extendoCommand.getExtendoOpenCommand;
import static org.firstinspires.ftc.teamcode.subsystems.deposit.getCatchPixel;
import static org.firstinspires.ftc.teamcode.subsystems.deposit.getPostIntakeState;
import static org.firstinspires.ftc.teamcode.subsystems.deposit.getRelease;


import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.commands.extendoCommand;
import org.firstinspires.ftc.teamcode.opModes.AutoBaseJava;
import org.firstinspires.ftc.teamcode.subsystems.clawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.deposit;
import org.firstinspires.ftc.teamcode.subsystems.linearSlides;

import dev.frozenmilk.mercurial.commands.groups.Parallel;
import dev.frozenmilk.mercurial.commands.groups.Sequential;
import dev.frozenmilk.mercurial.commands.util.Wait;
@Config
@Autonomous
public class java5spec extends AutoBaseJava {
    public java5spec() {super(Side.chamber);}

    public static Pose startPose = startingPoseChamber;
    public static Pose chamberPose1 = new Pose(-34, -5, Math.toRadians(180));
    public static Pose chamberPose2 = new Pose(-33, -7, Math.toRadians(180));
    public static Pose chamberPose3 = new Pose(-32.8, -2, Math.toRadians(180));
    public static Pose chamberPose4 = new Pose(-33.5, -4, Math.toRadians(180));
    public static Pose chamberPose5 = new Pose(-33.5, -6, Math.toRadians(180));
    public static Pose dragPose1 = new Pose(-45.2, -27, Math.toRadians(-45));
    public static Pose dragPose2 = new Pose(-44.3, -37.2, Math.toRadians(-45));
    public static Pose dragPose3 = new Pose(-40, -45, Math.toRadians(-45));
    public static Pose dragPoseTurn1 = new Pose(-45, -27, Math.toRadians(-45-110));
    public static Pose dragPoseTurn2 = new Pose(-44.5, -36.5, Math.toRadians(-45-110));
    public static Pose dragPoseTurn3 = new Pose(-40, -45, Math.toRadians(-45-110));
    public static Pose pickup1Pose = new Pose(-57, -38, Math.toRadians(0));
    public static Pose pickup12Pose = new Pose(-58.3, -38, Math.toRadians(0));
    public static Pose pickup13Pose = new Pose(-62, -38, Math.toRadians(0));
    public static Pose pickup2Pose = new Pose(-61.3, -25, Math.toRadians(180+80));
    public static Pose parkPose = new Pose(-47, -22, Math.toRadians(180+40));

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
        clawSubsystem.getClawRotationServo().setPosition(0.5);
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
            linearSlides.getGoToHighChamber(),
            followPath(scorePreload).with(
                new Sequential(
                    waitUntil(()->linearSlides.getPose()>500),
                    deposit.getArmOut()
                )
            ),
            deposit.getSlamSeq(),
            new Parallel(
                new Sequential(
                    waitUntil(()->follower.getCurrentTValue()>0.2),
                    extendoCommand.getExtendoOpenCommandAuto(),
                    clawSubsystem.getTurnLeft()
                ),
                instantCommand(()-> FollowerConstants.headingPIDFCoefficients.P = 0.8),
                followPath(getToDrag1)
            ),
            new Wait(0.25),
            clawSubsystem.getCloseClaw(),
            new Wait(0.1),
            instantCommand(()-> FollowerConstants.headingPIDFCoefficients.P = 1.9),
            turn(110).with(new Wait(1)),
            clawSubsystem.getOpenClaw(),
            instantCommand(()-> FollowerConstants.headingPIDFCoefficients.P = 0.75),
            followPath(getToDrag2),
            new Wait(0.25),
            clawSubsystem.getCloseClaw(),
            new Wait(0.1),
            instantCommand(()-> FollowerConstants.headingPIDFCoefficients.P = 1.9),
            turn(110).with(new Wait(1)),
            clawSubsystem.getOpenClaw(),
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
                    waitUntil(()-> !follower.isBusy()),
                    getCatchPixel().raceWith(slowX),
                    new Wait(0.2),
                    getPostIntakeState()
                )
            ),
            linearSlides.getGoToHighChamber(),
            followPath(scorePickup1),
            deposit.getSlamSeq(),
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
                    waitUntil(()-> !follower.isBusy()),
                    getCatchPixel().raceWith(slowX),
                    new Wait(0.2),
                    getPostIntakeState()
                )
            ),
            linearSlides.getGoToHighChamber(),
            followPath(specialHPIntake1score),
            deposit.getSlamSeq(),
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
                    waitUntil(()-> !follower.isBusy()),
                    getCatchPixel().raceWith(slowX),
                    new Wait(0.2),
                    getPostIntakeState()
                )
            ),
            linearSlides.getGoToHighChamber(),
            followPath(specialHPIntake1score),
            deposit.getSlamSeq(),
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
//            waitUntil(()->linearSlides.getPose()>1130),
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
//                waitUntil(()->linearSlides.getPose()>1130),
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
//                waitUntil(()->linearSlides.getPose()>1130),
//                deposit.getSlamSeq(),
//                new Wait(0.2),
//                followPath(pickup4),
//                followPath(pickup3),
//                followPath(scorePickup4),
            finishAuto
        ).schedule();

    }
    @Override
    public void myStop(){
        linearSlides.setStartingPose(linearSlides.getPose());
    }
}
