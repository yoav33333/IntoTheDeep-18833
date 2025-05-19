package org.firstinspires.ftc.teamcode.commands.util
import dev.frozenmilk.dairy.core.wrapper.Wrapper
import dev.frozenmilk.mercurial.commands.Command
import java.lang.Runnable
import java.util.Collections.emptySet

/**
 * a command that runs a lambda when scheduled
 */
class InstantCommand(val lambda: Runnable) : Command {

    /**
     * Executes the provided lambda when the command is initialized.
     */
    override fun initialise() {
        lambda.run()
    }

    /**
     * Does nothing during command execution.
     *
     * This method is intentionally left empty as the command completes immediately upon initialization.
     */
    override fun execute() {
    }

    /**
     * Called when the command ends or is interrupted; no action is performed.
     *
     * @param interrupted Indicates whether the command was interrupted.
     */
    override fun end(interrupted: Boolean) {
    }

    /**
     * Indicates that the command has completed immediately after initialization.
     *
     * @return Always returns true.
     */
    override fun finished(): Boolean {
        return true
    }

    override val requirements: Set<Any> = emptySet()
    override val runStates: Set<Wrapper.OpModeState> = setOf(Wrapper.OpModeState.INIT, Wrapper.OpModeState.ACTIVE)
    /**
 * Returns the string representation of this command.
 *
 * @return The string "InstantCommand".
 */
override fun toString() = "InstantCommand"
}