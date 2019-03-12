# Conventions

- These are the conventions that must be followed when editing this repository

## File Structure
- Lorem ipsum dolor sit amet

## Formatting
- Use of the `black` formatter ([https://github.com/ambv/black](https://github.com/ambv/black))
    - Install with `pip install black`

## Naming
- Snake case for variables
    - `drive_motor_left = TalonSRX(0)`
    - `veloicty_right = 10.0`
- Lower camel case for class methods and functions
    - `def setVelocity(self, velocity_left, velocity_right):`
    - `def getOdometry(self):`
- Upper camel case for class names
    - `class Robot(magicbot.MagicRobot):`
    - `class DriveTrain:`

## Units
- Use of the `pint` module
- Time: `seconds` (some native function use `ms`)
- Length: `inches` (some native functions use `feet` or `native units`)
- Angle: `degrees` (some native functions use `radians`)
- Coordinates:
    - x: `left/right`
    - y: `backwards/forwards`
    - theta: `counterclockwise starting from 0 facing forwards`