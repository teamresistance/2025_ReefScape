// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.OperatorConstants.JoystickType;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/** Class to configure possible joystick types and buttons. */
public class OperatorInput {
    public JoystickType joystickType;
    // Joysticks possible
    public static Joystick leftDrvrJS = new Joystick(0);
    public static Joystick rightDrvrJS = new Joystick(1);
    public static Joystick coDriverJS = new Joystick(2);
    public static Joystick reefSelector = new Joystick(3);
    public static Joystick neoJS = new Joystick(4);
    public static Joystick kybd1JS = new Joystick(5);
    public static Joystick kybd2JS = new Joystick(6);
    // JoystickButtons
    public JoystickButton lvl2Button;
    public JoystickButton lvl3Button;
    public JoystickButton lvl4Button;
    public JoystickButton selectBranchAndAddButton;
    public JoystickButton climbButton;
    // ReefselectorButtons
    public JoystickButton buttonA;
    public JoystickButton buttonB;
    public JoystickButton buttonC;
    public JoystickButton buttonD;
    public JoystickButton buttonE;
    public JoystickButton buttonF;
    public JoystickButton buttonRL;
    public JoystickButton button4;
    public JoystickButton button3;
    public JoystickButton button2_1;

    /**
     * Constructor to configure buttons for joystick type passed
     * 
     * @param js_Type 0=Normal 3 JS's, 1=Neopad, 2=Keyboard
     */
    public OperatorInput(JoystickType jsType) {
        // Configure the button trigger bindings
        JoystickType defaultConfigJS = jsType; // 2; // 0=Normal 3 JS's, 1=Neopad, 2=Keyboard
        joystickType = jsType;
        switch (defaultConfigJS) {
            case k3Joysticks:
                config3Joysticks();
                break;
            case k2JoysticksAndReefSelector:
                config2JoysticksAndReefSelector();
                break;
            case kNeo:
                configNeo();
                break;
            case kKybd1:
                configKybd();
                break;
            default:
                config3Joysticks();
                System.out.println("Bad joystick configuration: " + defaultConfigJS);
        }
    }

    /** Configue all buttons defined for 3 JS's */
    private void config3Joysticks() {
        // Create a JSB for each level
        lvl2Button = new JoystickButton(coDriverJS, OperatorConstants.klvl2Button_3Joysticks_ID);
        lvl3Button = new JoystickButton(coDriverJS, OperatorConstants.klvl3Button_3Joysticks_ID);
        lvl4Button = new JoystickButton(coDriverJS, OperatorConstants.klvl4Button_3Joysticks_ID);
        selectBranchAndAddButton = new JoystickButton(coDriverJS,
                OperatorConstants.kSelectBranchAndAddButton_3Joysticks_ID);
        climbButton = new JoystickButton(coDriverJS, OperatorConstants.kclimbButton_3Joysticks_ID);
    }

    private void config2JoysticksAndReefSelector() {
        // Create a JSB for each level
        buttonA = new JoystickButton(reefSelector, 1);
        buttonB = new JoystickButton(reefSelector, 2);
        buttonC = new JoystickButton(reefSelector, 3);
        buttonD = new JoystickButton(reefSelector, 4);
        buttonE = new JoystickButton(reefSelector, 5);
        buttonF = new JoystickButton(reefSelector, 6);

        buttonRL = new JoystickButton(reefSelector, 7);

        button4 = new JoystickButton(reefSelector, 8);
        button3 = new JoystickButton(reefSelector, 9);
        button2_1 = new JoystickButton(reefSelector, 10);
        climbButton = new JoystickButton(coDriverJS, OperatorConstants.kclimbButton_2JoysticksAndReefSelector_ID);
    }

    /** Configue all buttons defined for a Niteno Pad */
    private void configNeo() {
        // Create a JSB for each level
        lvl2Button = new JoystickButton(neoJS, OperatorConstants.klvl2Button_Neo_ID);
        lvl3Button = new JoystickButton(neoJS, OperatorConstants.klvl3Button_Neo_ID);
        lvl4Button = new JoystickButton(neoJS, OperatorConstants.klvl4Button_Neo_ID);
        selectBranchAndAddButton = new JoystickButton(neoJS, OperatorConstants.kSelectBranchAndAddButton_Neo_ID);
        climbButton = new JoystickButton(coDriverJS, OperatorConstants.kclimbButton_Neo_ID);
    }

    /** Configue all buttons defined to use the keyboard */
    private void configKybd() {
        // Create a JSB for each level
        lvl2Button = new JoystickButton(kybd1JS, OperatorConstants.klvl2Button_Kybd1_ID);
        lvl3Button = new JoystickButton(kybd1JS, OperatorConstants.klvl3Button_Kybd1_ID);
        lvl4Button = new JoystickButton(kybd1JS, OperatorConstants.klvl4Button_Kybd1_ID);
        selectBranchAndAddButton = new JoystickButton(kybd1JS, OperatorConstants.kSelectBranchAndAddButton_Kybd1_ID);
    }
}
