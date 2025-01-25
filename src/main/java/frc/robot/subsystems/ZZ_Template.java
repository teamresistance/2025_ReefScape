package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Timer;

/** an example of timed statemachine implementation into command base */
public class ZZ_Template extends SubsystemBase {
  // hdw defintions:

  // joystick buttons:

  private static final Timer stateTmr = new Timer(.05); // Timer for state machine
  // variables:
  private static int state; // ???? state machine. 0=Off by pct, 1=On by velocity, RPM

  /** Initialize ???? stuff. Called from telopInit (maybe robotInit(?)) in Robot.java */
  public ZZ_Template() {
    sdbInit();
    cmdUpdate(0.0, false, false); // Make sure all is off
    state = 0; // Start at state 0
  }

  /**
   * Update ????. Called from teleopPeriodic in robot.java.
   *
   * <p>Determine any state that needs to interupt the present state, usually by way of a JS button
   * but can be caused by other events.
   */
  private static void smUpdate() { // State Machine Update
    switch (state) {
      case 0: // Everything is off
        cmdUpdate(0.0, false, false);
        stateTmr.hasExpired(0.05, state); // Initialize timer for covTrgr. Do nothing.
        break;
      case 1: // Do sumpthin and wait for action
        cmdUpdate(100.0, true, false);
        if (stateTmr.hasExpired(0.05, state)) state++;
        break;
      case 2: // Shutdown and wait for action then go to 0
        cmdUpdate(50.0, false, true);
        if (stateTmr.hasExpired(0.05, state)) state = 0;
        break;
      default: // all off
        cmdUpdate(0.0, false, false);
        System.out.println("Bad sm state:" + state);
        break;
    }
  }

  /**
   * Issue spd setting as rpmSP if isVelCmd true else as percent cmd.
   *
   * @param select_low - select the low goal, other wise the high goal
   * @param left_trigger - triggers the left catapult
   * @param right_trigger - triggers the right catapult
   */
  private static void cmdUpdate(double dblSig, boolean trigger1, boolean trigger2) {
    // Check any safeties, mod passed cmds if needed.
    // Send commands to hardware
  }

  /*-------------------------  SDB Stuff --------------------------------------
  /**Initialize sdb */
  private static void sdbInit() {
    // Put stuff here on the sdb to be retrieved from the sdb later
    // SmartDashboard.putBoolean("ZZ_Template/Sumpthin", sumpthin.get());
  }

  /** Update the Smartdashboard. */
  private static void sdbUpdate() {
    // Put stuff to retrieve from sdb here.  Must have been initialized in sdbInit().
    // sumpthin = SmartDashboard.getBoolean("ZZ_Template/Sumpthin", sumpthin.get());

    // Put other stuff to be displayed here
    SmartDashboard.putNumber("ZZ_Template/state", state);
  }

  /**
   * Probably shouldn't use this bc the states can change. Use statuses.
   *
   * @return - present state of Shooter state machine.
   */
  public static int getState() {
    return state;
  }

  // ----------------- Shooter statuses and misc.-----------------

  /**
   * @return If the state machine is running, not idle.
   */
  public static boolean getStatus() {
    return state != 0; // This example says the sm is runing, not idle.
  }

  /**
   * Update ????. Called from teleopPeriodic in robot.java.
   *
   * <p>Determine any state that needs to interupt the present state, usually by way of a JS button
   * but can be caused by other events.
   */
  @Override
  public void periodic() {
    // Add code here to start state machine or override the sm sequence
    smUpdate();
    sdbUpdate();
  }
}
