package org.firstinspires.ftc.teamcode.opModes.auto;

import static org.firstinspires.ftc.teamcode.commands.extendoCommand.getExtendoOpenCommand;
import static org.firstinspires.ftc.teamcode.subsystems.deposit.getCatchPixel;
import static org.firstinspires.ftc.teamcode.subsystems.deposit.getPostIntakeState;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.commands.extendoCommand;
import org.firstinspires.ftc.teamcode.opModes.AutoBaseJava;
import org.firstinspires.ftc.teamcode.subsystems.armClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.clawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.deposit;
import org.firstinspires.ftc.teamcode.subsystems.linearSlides;
import org.firstinspires.ftc.teamcode.util.InstantCommand;
import org.firstinspires.ftc.teamcode.util.RunNonBlocking;
import org.firstinspires.ftc.teamcode.util.WaitUntil;

import dev.frozenmilk.mercurial.commands.groups.Parallel;
import dev.frozenmilk.mercurial.commands.groups.Sequential;
import dev.frozenmilk.mercurial.commands.util.Wait;

@Config
@Autonomous
public class java5specNewPath extends AutoBaseJava {
    public java5specNewPath() {super(Side.chamber);}

    public Pose startPose = this.startingPoseChamber;
    public Pose chamberPose1 = new Pose(-34, -2, Math.toRadians(180));
    public Pose chamberPose2 = new Pose(-34.4-1.05, -3, Math.toRadians(180));
    public Pose chamberPose3 = new Pose(-34.4-1.05, -4, Math.toRadians(180));
    public Pose chamberPose4 = new Pose(-34.8-1.05, -5, Math.toRadians(180));
    public Pose chamberPose5 = new Pose(-35.1-1.05 , -7, Math.toRadians(180));
    public Pose dragPose1 = new Pose(-45.2, -26, Math.toRadians(-46));
    public Pose dragPose2 = new Pose(-44.3, -36.2, Math.toRadians(-46));
    public Pose dragPose3 = new Pose(-38, -41.5, Math.toRadians(-40));
    public Pose dragPoseTurn1 = new Pose(-45.2, -27, Math.toRadians(-45));
    public Pose dragPoseTurn2 = new Pose(-44.3, -37.2, Math.toRadians(-45));
    public Pose dragPoseTurn3 = new Pose(-45, -42.5, Math.toRadians(-45-90));
    public Pose pickup1Pose = new Pose(-52.3, -34, Math.toRadians(-135));
    public Pose pickup12Pose = new Pose(-52.3, -34, Math.toRadians(-135));
    public Pose pickup13Pose = new Pose(-62, -38, Math.toRadians(0));
    public Pose pickup2Pose = new Pose(-61.3, -25, Math.toRadians(180+80));
    public Pose parkPose = new Pose(-47, -22, Math.toRadians(180+40));

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
//        follower.setMaxPower(0.7);
        scorePreload = makeLinePath(startPose, chamberPose1);
        getToDrag1 = makeLinePath(chamberPose1, dragPose1);
        turnDrag1 = makeLinePath(dragPose1, dragPoseTurn1);
        getToDrag2 = makeLinePath(dragPoseTurn1, dragPose2);
        turnDrag2 = makeLinePath(dragPose2, dragPoseTurn2);
        getToDrag3 = makeLinePath(dragPoseTurn2, dragPose3);
        turnDrag3 = makeLinePath(dragPose3, dragPoseTurn3);
//        specialHPIntake = makeLinePath(dragPoseTurn3, pickup1Pose);
        specialHPIntake = makeLinePath(dragPoseTurn3, pickup1Pose);
        specialHPIntake1 = makeLinePath(chamberPose2, pickup12Pose);
        park = makeLinePath(chamberPose5, parkPose);
        specialHPIntake1score = makeLinePath(pickup12Pose, chamberPose2);
//        dragToHP = makeCurvePath(chamberPose1, dragPose1, dragPoseTurn1, dragPose2, dragPoseTurn2, dragPose3, dragPoseTurn3,pickup1Pose);
        scorePickup1 = makeLinePath(pickup1Pose, chamberPose2);
        scorePickup2 = makeLinePath(pickup1Pose, chamberPose3);
        scorePickup3 = makeLinePath(pickup1Pose, chamberPose4);
        scorePickup4 = makeLinePath(pickup1Pose, chamberPose5);
        pickup1 = makeLinePath(chamberPose2, pickup1Pose);
        pickup2 = makeLinePath(chamberPose3, pickup1Pose);
        pickup3 = makeLinePath(chamberPose4, pickup1Pose);
        pickup4 = makeLinePath(chamberPose5, pickup1Pose);
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
                clawSubsystem.getCloseClaw(),
                clawSubsystem.getResetAngleClaw(),

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
                turn(95).with(new Wait(0.9)),
                armClawSubsystem.getArmUp(),

//            clawSubsystem.getOpenClaw(),
//            instantCommand(()-> FollowerConstants.headingPIDFCoefficients.P = 0.75),
                new RunNonBlocking(new Sequential(
                        new WaitUntil(()->follower.getCurrentTValue()>0.9),
                        armClawSubsystem.getExtendoPushState()
                )),
                followPath(getToDrag2),

                turn(95).with(new Wait(0.9)),
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
                    new Wait(0.1),
                    linearSlides.getGoToLowChamberNoRC()
                ),
                followPath(specialHPIntake),
//                new Sequential(
//                    deposit.getIntakeCommand(),
//                    new Wait(0.1),
//                    deposit.getRelease(),
//                    new WaitUntil(()-> !follower.isBusy()),
//                    getCatchPixel().raceWith(slowX),
//                    new RunNonBlocking(
//                            new Sequential(new Wait(0.2),
//                    getPostIntakeState())
//                ))
                    extendoCommand.getNonExtendoOpenCommandAuto()
            ),
            clawSubsystem.getCloseClaw(),
            linearSlides.getGoToHighChamber(),
                new InstantCommand(()->follower.setMaxPower(0.7)),
                new RunNonBlocking(
                        new Sequential(
                                new WaitUntil(()->linearSlides.getPose()>1000),
                                deposit.getArmOut()
//                                new Wait(0.1),
//                                deposit.getQuickRCSimple()
                        )
                ),
                extendoCommand.getExtendoCloseCommandAuto().with(
                        followPath(scorePickup1)),
                new WaitUntil(()->linearSlides.getPose()>38000),
                new InstantCommand(()->follower.setMaxPower(1)),
                deposit.getSlamSeq(),
                new Parallel(
                    new Sequential(
                        new Wait(0.1),
                        linearSlides.getGoToLowChamberNoRC()
                    ),
                    followPath(pickup1),
                        extendoCommand.getNonExtendoOpenCommandAuto()
                ),
                clawSubsystem.getCloseClaw(),
                linearSlides.getGoToHighChamber(),
                new InstantCommand(()->follower.setMaxPower(0.9)),
                new RunNonBlocking(
                        new Sequential(
                                new WaitUntil(()->linearSlides.getPose()>1000),
                                deposit.getArmOut()
//                                new Wait(0.1),
//                                deposit.getQuickRCSimple()
                        )
                ),
                extendoCommand.getExtendoCloseCommandAuto().with(
                        followPath(scorePickup2)),
                new WaitUntil(()->linearSlides.getPose()>38000),
                new InstantCommand(()->follower.setMaxPower(1)),

                deposit.getSlamSeq(),
                new Parallel(
                new Sequential(
                    new Wait(0.1),
                    linearSlides.getGoToLowChamberNoRC()
                ),
                followPath(pickup2),
                        extendoCommand.getNonExtendoOpenCommandAuto()
                ),
                clawSubsystem.getCloseClaw(),
                linearSlides.getGoToHighChamber(),
                new InstantCommand(()->follower.setMaxPower(0.9)),
                new RunNonBlocking(
                        new Sequential(
                                new WaitUntil(()->linearSlides.getPose()>1000),
                                deposit.getArmOut()
//                                new Wait(0.1),
//                                deposit.getQuickRCSimple()
                        )
                ),
                extendoCommand.getExtendoCloseCommandAuto().with(
                        followPath(scorePickup3)),
                new WaitUntil(()->linearSlides.getPose()>38000),
                new InstantCommand(()->follower.setMaxPower(1)),

                deposit.getSlamSeq(),
            new Parallel(
                new Sequential(
                    new Wait(0.1),
                    linearSlides.getGoToLowChamberNoRC()
                ),
                followPath(pickup3),
                    extendoCommand.getNonExtendoOpenCommandAuto()
            ),
                clawSubsystem.getCloseClaw(),
                linearSlides.getGoToHighChamber(),
                new InstantCommand(()->follower.setMaxPower(0.9)),
                new RunNonBlocking(
                        new Sequential(
                                new WaitUntil(()->linearSlides.getPose()>1000),
                                deposit.getArmOut()
//                                new Wait(0.1),
//                                deposit.getQuickRCSimple()
                        )
                ),
                extendoCommand.getExtendoCloseCommandAuto().with(
                        followPath(scorePickup4)),

                new WaitUntil(()->linearSlides.getPose()>38000),
                new InstantCommand(()->follower.setMaxPower(1)),
                deposit.getSlamSeq(),
                new Parallel(
//                    new Sequential(
//                            new Wait(0.2),
//                        linearSlides.getGoToLowChamberNoRC()
//                    ),
                    new Wait(0.2).then(getExtendoOpenCommand()),
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
        ).schedule();

    }
    @Override
    public void myStop(){
        linearSlides.setStartingPose(linearSlides.getPose());
    }
}
