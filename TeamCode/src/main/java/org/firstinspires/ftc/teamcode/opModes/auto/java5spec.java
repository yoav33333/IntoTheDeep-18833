package org.firstinspires.ftc.teamcode.opModes.auto;

import static org.firstinspires.ftc.teamcode.commands.extendoCommand.getExtendoCloseCommandAuto;
import static org.firstinspires.ftc.teamcode.commands.extendoCommand.getExtendoOpenCommand;
import static org.firstinspires.ftc.teamcode.subsystems.deposit.getCatchPixel;
import static org.firstinspires.ftc.teamcode.subsystems.deposit.getPostIntakeState;
import static org.firstinspires.ftc.teamcode.subsystems.deposit.getRelease;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.commands.extendoCommand;
import org.firstinspires.ftc.teamcode.opModes.AutoBaseJava;
import org.firstinspires.ftc.teamcode.subsystems.BulkReads;
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
public class java5spec extends AutoBaseJava {
    public java5spec() {super(Side.chamber);}
    public static double offset = -0.6;
    public MultipleTelemetry tele = new MultipleTelemetry(telemetry,
            FtcDashboard.getInstance().getTelemetry());
    @Override
    public void myLoop(){
        tele.addData("current Control", BulkReads.modules.get(0).getCurrent(CurrentUnit.AMPS));
        tele.addData("current Expansion", BulkReads.modules.get(1).getCurrent(CurrentUnit.AMPS));
        tele.addData("current sum", BulkReads.modules.get(1).getCurrent(CurrentUnit.AMPS)+BulkReads.modules.get(1).getCurrent(CurrentUnit.AMPS));

        tele.update();
    }
    @Override
    public void myInitLoop() {
        tele.addData("current Control", BulkReads.modules.get(0).getCurrent(CurrentUnit.AMPS));
        tele.addData("current Expansion", BulkReads.modules.get(1).getCurrent(CurrentUnit.AMPS));
        tele.addData("current sum", BulkReads.modules.get(1).getCurrent(CurrentUnit.AMPS)+BulkReads.modules.get(1).getCurrent(CurrentUnit.AMPS));

        tele.update();
    }
    public Pose startPose = this.startingPoseChamber;
    public Pose chamberPose1 = new Pose(-35.5, -3, Math.toRadians(180));
    public Pose chamberPose2 = new Pose(-32.0+offset, -5+0.8, Math.toRadians(179));
    public Pose chamberPose3 = new Pose(-29.6+offset, -9+0.8, Math.toRadians(179));
    public Pose chamberPose4 = new Pose(-27.5+offset, -12+0.8, Math.toRadians(179));
    public Pose chamberPose5 = new Pose(-24.5+offset, -14+0.8, Math.toRadians(179));
    public Pose dragPose1 = new Pose(-35, -22.0, Math.toRadians(-62));
    public Pose dragPose2 = new Pose(-37, -31.4, Math.toRadians(-60));
    public Pose dragPose3 = new Pose(-37, -40.7, Math.toRadians(-60));
    public Pose dragPoseTurn1 = new Pose(-40.2, -24, Math.toRadians(-65-75));
    public Pose dragPoseTurn2 = new Pose(-40.3, -35.2, Math.toRadians(-65-75));
    public Pose dragPoseTurn3 = new Pose(-45, -42.1, Math.toRadians(-65-65));
    public Pose pickup1Pose = new Pose(-57-2.6, -39-2, Math.toRadians(0));
    public Pose pickup2Pose = new Pose(-55-2.6, -40-2, Math.toRadians(0));
    public Pose pickup3Pose = new Pose(-53-2.6, -41-2, Math.toRadians(0));
    public Pose pickup4Pose = new Pose(-51-2.6, -42-2, Math.toRadians(0));
    public Pose pickup12Pose = new Pose(-59, -42, Math.toRadians(0));
    public Pose pickup13Pose = new Pose(-62, -38, Math.toRadians(0));
//    public Pose pickup2Pose = new Pose(-61.3, -25, Math.toRadians(180+80));
    public Pose parkPose = new Pose(-47, -29, Math.toRadians(180+40));

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
    public static double power = 1.0;
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
        specialHPIntake = makeSpinHalfWayPath(dragPoseTurn3, pickup1Pose);
        specialHPIntake1 = makeLinePath(chamberPose2, pickup12Pose);
        park = makeLinePath(chamberPose5, parkPose);
        specialHPIntake1score = makeLinePath(pickup12Pose, chamberPose2);
//        dragToHP = makeCurvePath(chamberPose1, dragPose1, dragPoseTurn1, dragPose2, dragPoseTurn2, dragPose3, dragPoseTurn3,pickup1Pose);
        scorePickup1 = makeSpinHalfWayPath(pickup1Pose, chamberPose2);
        scorePickup2 = makeSpinHalfWayPath(pickup2Pose, chamberPose3);
        scorePickup3 = makeSpinHalfWayPath(pickup3Pose, chamberPose4);
        scorePickup4 = makeSpinHalfWayPath(pickup4Pose, chamberPose5);
        pickup1 = makeSpinHalfWayPath(chamberPose2, pickup2Pose);
        pickup2 = makeSpinHalfWayPath(chamberPose3, pickup3Pose);
        pickup3 = makeSpinHalfWayPath(chamberPose4, pickup4Pose);
//        pickup4 = makeLinePath(chamberPose5, pickup12Pose);
//        park = makeLinePath(basketScore, parkPose);
//        clawSubsystem.getClawRotationServo().setPosition(0.5);
        deposit.armOutHalf();
        deposit.closeClawRaw();
        clawSubsystem.openClaw();
        follower.setMaxPower(power);
    }

    @Override
    public void myStart() {
        linearSlides.setPose(0);

//        deposit.armOut();
//        linearSlides.getRunToPosition().schedule();
        new Sequential(
//                clawSubsystem.getTurnRight(),

                new RunNonBlocking(linearSlides.getRunToPosition()),
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
                    new RunNonBlocking(linearSlides.getCloseSlidesAuto()),
                new Sequential(
                    new WaitUntil(()->follower.getCurrentTValue()>0.6),
                    extendoCommand.getExtendoOpenCommandAutoPush(),
                    armClawSubsystem.getArmUp()
//                    clawSubsystem.getTurnLeft()
                ),
                new RunNonBlocking(new Sequential(
                        new WaitUntil(()->follower.getCurrentTValue()>0.99),
                        armClawSubsystem.getExtendoPushState()
                        )),
//                instantCommand(()-> FollowerConstants.headingPIDFCoefficients.P = 0.8),
                followPath(getToDrag1)
            ),
//            new Wait(0.25),
//            clawSubsystem.getCloseClaw(),
//            new Wait(0.1),
//                instantCommand(()-> FollowerConstants.headingPIDFCoefficients.P = 1.9),
//                new InstantCommand(()-> FollowerConstants.headingPIDFCoefficients.P = 5),
                followPath(turnDrag1),
//                turn(90).with(new Wait(0.8)),
//                new InstantCommand(()-> FollowerConstants.headingPIDFCoefficients.P = 1.94),
                armClawSubsystem.getArmUp(),

//            clawSubsystem.getOpenClaw(),
//            instantCommand(()-> FollowerConstants.headingPIDFCoefficients.P = 0.75),
                new RunNonBlocking(new Sequential(
                        new WaitUntil(()->follower.getCurrentTValue()>0.99),
                        armClawSubsystem.getExtendoPushState()
                )),
                followPath(getToDrag2),
//                new InstantCommand(()-> FollowerConstants.headingPIDFCoefficients.P = 5),
//
//                turn(90).with(new Wait(0.8)),
//                new InstantCommand(()-> FollowerConstants.headingPIDFCoefficients.P = 1.94),
                followPath(turnDrag2),

//            new Wait(0.25),
//            clawSubsystem.getCloseClaw(),
//            new Wait(0.1),
//            instantCommand(()-> FollowerConstants.headingPIDFCoefficients.P = 1.9),
//                followPath(turnDrag2),
                armClawSubsystem.getArmUp(),
                new RunNonBlocking(new Sequential(
                        new WaitUntil(()->follower.getCurrentTValue()>0.99),
                        armClawSubsystem.getExtendoPushState()
                )),
                followPath(getToDrag3),
//            new Wait(0.25),
//            clawSubsystem.getCloseClaw(),
//            new Wait(0.1),
//            instantCommand(()-> FollowerConstants.headingPIDFCoefficients.P = 1.9),
            followPath(turnDrag3),
//            clawSubsystem.getOpenClaw(),
                new RunNonBlocking(

            extendoCommand.getExtendoCloseCommandSimple()),
            new Wait(0.05),
                new Parallel(
                new Sequential(
                    new Wait(0.2),
                    linearSlides.getGoToLowChamberNoRC()
                ),
                followPath(specialHPIntake).raceWith(new Sequential(new Wait(0.15)),getCatchPixel()),
                new Sequential(
                    new Wait(0.6),
                    deposit.getIntakeCommand(),
                    deposit.getRelease(),
                    new WaitUntil(()-> !follower.isBusy()),
                    getCatchPixel().raceWith(slowX).raceWith(new Sequential(new Wait(0.4), new InstantCommand(deposit::closeClaw))),
                    new RunNonBlocking(
                            new Sequential(new Wait(0.2),
                    getPostIntakeState())
                )
                    )
            ),
            linearSlides.getGoToHighChamberUp(),
                followPath(scorePickup1),
//                new Wait(0.1),
                deposit.getSlamArmDown(),
//                new Wait(0.1),

                new Parallel(
                    new Sequential(
                        new Wait(0.2),
                        linearSlides.getGoToLowChamberNoRC()
                    ),
                    followPath(pickup1).raceWith(new Sequential(new Wait(0.15)),getCatchPixel()),
                        new Sequential(
                                new Wait(0.6),
                                deposit.getIntakeCommand(),
                                deposit.getRelease(),
                                new WaitUntil(()-> !follower.isBusy()),
                                getCatchPixel().raceWith(slowX).raceWith(new Sequential(new Wait(0.4), new InstantCommand(deposit::closeClaw))),
                                new RunNonBlocking(
                                        new Sequential(new Wait(0.2),
                                                getPostIntakeState())
                                )
                    )),
            linearSlides.getGoToHighChamberUp(),
            followPath(scorePickup2),
//                new Wait(0.1),
                deposit.getSlamArmDown(),
//                new Wait(0.1),

                new Parallel(
                new Sequential(
                    new Wait(0.2),
                    linearSlides.getGoToLowChamberNoRC()
                ),
                followPath(pickup2).raceWith(new Sequential(new Wait(0.15)),getCatchPixel()),
                new Sequential(
                        new Wait(0.6),
                        deposit.getIntakeCommand(),
                        deposit.getRelease(),
                    new WaitUntil(()-> !follower.isBusy()),
                    getCatchPixel().raceWith(slowX).raceWith(new Sequential(new Wait(0.4), new InstantCommand(deposit::closeClaw))),
                        new RunNonBlocking(
                                new Sequential(new Wait(0.2),
                                        getPostIntakeState())
                )
                )),
            linearSlides.getGoToHighChamberUp(),
            followPath(scorePickup3),
//                new Wait(0.1),
                deposit.getSlamArmDown(),
//                new Wait(0.1),

                new Parallel(
                new Sequential(
                    new Wait(0.2),
                    linearSlides.getGoToLowChamberNoRC()
                ),
                followPath(pickup3).raceWith(new Sequential(new Wait(0.15)),getCatchPixel()),
                new Sequential(
                        new Wait(0.4),
                        deposit.getIntakeCommand(),
                        deposit.getRelease(),
                    new WaitUntil(()-> !follower.isBusy()),
                    getCatchPixel().raceWith(slowX).raceWith(new Sequential(new Wait(0.8), new InstantCommand(deposit::closeClaw))),
                        new RunNonBlocking(
                                new Sequential(new Wait(0.2),
                                        getPostIntakeState()))
                )
            ),
            linearSlides.getGoToHighChamberUp(),
            followPath(scorePickup4),
//                new Wait(0.1),
                deposit.getSlamArmDown(),
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
