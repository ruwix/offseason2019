# Conventions

- These are the conventions that must be followed when editing this repository

## File Structure
- Lorem ipsum dolor sit amet

## Formatting
- `format` script in source folder
- `autoflake` for removing unused imports  ([https://github.com/myint/autoflake](https://github.com/myint/autoflake))
    - Install with `pip install autoflake`
- `isort` for organizing imports  ([https://github.com/timothycrosley/isort](https://github.com/timothycrosley/isort))
    - Install with `pip install isort`
- `black` for formatting ([https://github.com/ambv/black](https://github.com/ambv/black))
    - Install with `pip install black`

## Naming
- Snake case for variables
    - `drive_motor_left = TalonSRX(0)`
    - `veloicty_right = 10.0`
- Lower camel case for class methods and functions
    - `def setVelocity(self, velocity_left, velocity_right):`
    - `def boundRadians():`
- Upper camel case for class names
    - `class Robot(magicbot.MagicRobot):`
    - `class DriveTrain:`

## Units
- Time: `seconds` (some native function use `ms`)
- Length: `meters` (some native functions use `feet`)
- Angle: `radians` (some native functions use `degrees`; the `config.json` file uses degrees; paths are saved with `degrees` but are converted to `radians` when imported)
- Weight: `kilograms`
- Coordinates:
    - x: `left/right`
    - y: `backwards/forwards`
    - theta: `counterclockwise starting from 0 facing forwards`