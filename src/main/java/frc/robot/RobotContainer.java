package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.util.util;
import frc.robot.autos.*;
import frc.robot.commands.*;
import frc.robot.commands.GrapAndPlace.GrapOrPlace;
import frc.robot.commands.GrapAndPlace.GrapOrPlaceInstent;
import frc.robot.commands.GrapAndPlace.SetARMpostionToPlace;
import frc.robot.commands.GrapAndPlace.placeCommandGroop;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    /* Controllers */
    private final Joystick m_driver = new Joystick(0);
    private final Joystick m_HelperDriverController = new Joystick(1);

    //pov
    POVButton d_Uppov = new POVButton(m_driver, 0);
    POVButton d_Rahgtpov = new POVButton(m_driver, 90);
    POVButton d_Downpov = new POVButton(m_driver, 180);
    POVButton d_Leftpov = new POVButton(m_driver, 270);

    POVButton h_Uppov = new POVButton(m_HelperDriverController, 0);
    POVButton h_Rahgtpov = new POVButton(m_HelperDriverController, 90);
    POVButton h_Downpov = new POVButton(m_HelperDriverController, 180);
    POVButton h_Leftpov = new POVButton(m_HelperDriverController, 270);

    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(m_driver, XboxController.Button.kRightBumper.value);
    private final JoystickButton robotCentric = new JoystickButton(m_driver, XboxController.Button.kLeftBumper.value);
    private final JoystickButton kY = new JoystickButton(m_driver, XboxController.Button.kY.value);
    private final JoystickButton ka = new JoystickButton(m_driver, XboxController.Button.kA.value);
    private final JoystickButton kx = new JoystickButton(m_driver, XboxController.Button.kX.value);
    private final JoystickButton kB = new JoystickButton(m_driver, XboxController.Button.kB.value);

    private final Trigger speedaut = new Trigger(() -> m_driver.getRawAxis(2) > 0.1);

    /* System Buttons */
    private final JoystickButton hkY = new JoystickButton(m_driver, XboxController.Button.kY.value);
    private final JoystickButton hka = new JoystickButton(m_driver, XboxController.Button.kA.value);
    private final JoystickButton hkx = new JoystickButton(m_driver, XboxController.Button.kX.value);
    private final JoystickButton hkB = new JoystickButton(m_driver, XboxController.Button.kB.value);

    private final Trigger pik = new Trigger(() -> m_driver.getRawAxis(2) > 0.1);
    private final Trigger aut = new Trigger(() -> m_driver.getRawAxis(3) > 0.1);

    /* Subsystems */
    private final Swervesubsystem s_Swerve = new Swervesubsystem();
    private final Grappersubsystem m_grapper = new Grappersubsystem();
    private final ARMsubsystem m_arm = new ARMsubsystem();
    private final limelightSubSystem m_Limelight = new limelightSubSystem();


    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        s_Swerve.setDefaultCommand(
            new TeleopSwerveCommand(
                s_Swerve, 
                () -> -m_driver.getRawAxis(translationAxis), 
                () -> -m_driver.getRawAxis(strafeAxis), 
                () -> -m_driver.getRawAxis(rotationAxis), 
                () -> robotCentric.getAsBoolean(),
                () -> speedaut.getAsBoolean()
            )

        );
        
        // Configure the button bindings
        configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        
        /* Driver Buttons */
        zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));

        
        /*helper Draiver */

        hkB.onTrue(Commands.runOnce(()->m_arm.setposison(75)));

        hkY.onTrue(Commands.runOnce(()->m_arm.setposison(69)));

        pik.onTrue(Commands.run(()->m_grapper.speed(0.5)));

        aut.onTrue(Commands.run(()->m_grapper.speed(-0.5)));


        /*Bol Buttons */
        kY.onTrue(Commands.runOnce(()-> m_grapper.changstate()));

        /* System Buttons */
        double y = util.kalculatdisrtans(m_Limelight.targetX());
    }

    public Swervesubsystem getSwerveSubsystem() {
        return s_Swerve;
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        return new autoPathPlanner(s_Swerve, m_arm);
    }
}
