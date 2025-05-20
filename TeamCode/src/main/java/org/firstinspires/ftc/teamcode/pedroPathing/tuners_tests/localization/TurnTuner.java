package org.firstinspires.ftc.teamcode.pedroPathing.tuners_tests.localization;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.pedropathing.localization.PoseUpdater;
import com.pedropathing.util.DashboardPoseTracker;
import com.pedropathing.util.Drawing;

import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;

/**
 * This is the TurnTuner OpMode. This tracks the turning movement of the Robot and displays the
 * necessary ticks to inches multiplier. This displayed multiplier is what's necessary to scale the
 * Robot's current angle in ticks to the specified angle in radians. So, to use this, run the
 * tuner, then pull/push the Robot to the specified angle using a protractor or lines on the ground.
 * When you're at the end of the angle, record the ticks to inches multiplier. Feel free to run
 * multiple trials and average the results. Then, input the multiplier into the turning ticks to
 * radians in your localizer of choice.
 * You can adjust the target angle on FTC Dashboard: 192/168/43/1:8080/dash
 *
 * @author Anyi Lin - 10158 Scott's Bots
 * @version 1.0, 5/6/2024
 */
@Config
@Autonomous(name = "Turn Localizer Tuner", group = ".Localization")
public class TurnTuner extends OpMode {
    private PoseUpdater poseUpdater;
    private DashboardPoseTracker dashboardPoseTracker;

    private Telemetry telemetryA;

    public static double ANGLE = 2 * Math.PI;

    /**
     * Initializes the pose updater, dashboard pose tracker, and combined telemetry for turn calibration.
     *
     * Sets up hardware mappings, configures telemetry output to both the driver station and FTC Dashboard, and displays initial instructions and the Robot's pose for the turn tuning process.
     */
    @Override
    public void init() {
        Constants.setConstants(FConstants.class, LConstants.class);
        poseUpdater = new PoseUpdater(hardwareMap);

        dashboardPoseTracker = new DashboardPoseTracker(poseUpdater);

        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetryA.addLine("Turn your Robot " + ANGLE + " radians. Your turn ticks to inches will be shown on the telemetry.");
        telemetryA.update();

        Drawing.drawRobot(poseUpdater.getPose(), "#4CAF50");
        Drawing.sendPacket();
    }

    /**
     * Updates the Robot's pose estimate and displays the current heading and calculated turn multiplier on the FTC Dashboard.
     *
     * This method refreshes the Robot's pose, outputs telemetry data including the total heading and the computed multiplier for converting turn ticks to inches, and visualizes the Robot's pose and history on the dashboard.
     */
    @Override
    public void loop() {
        poseUpdater.update();

        telemetryA.addData("total angle", poseUpdater.getTotalHeading());
        telemetryA.addLine("The multiplier will display what your turn ticks to inches should be to scale your current angle to " + ANGLE + " radians.");
        telemetryA.addData("multiplier", ANGLE / (poseUpdater.getTotalHeading() / poseUpdater.getLocalizer().getTurningMultiplier()));

        Drawing.drawPoseHistory(dashboardPoseTracker, "#4CAF50");
        Drawing.drawRobot(poseUpdater.getPose(), "#4CAF50");
        Drawing.sendPacket();
    }
}
