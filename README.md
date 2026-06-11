# dynamixel-sdk-position-velocity-control

Position and velocity control of the Dynamixel Pro L54-50-S290-R under no-load conditions using the built-in PI controller.

This project is a basic introduction to working with Dynamixel products. Being able to set and track system states (position and velocity) is a useful skill when working with real systems and building more advanced controllers with a load attached to the servo in a full closed-loop configuration.

Two Python files are included: one for position control and one for velocity control.

## Requirements

Install the following libraries before running:

```
pip install dynamixel_sdk matplotlib
```

## Notes

- If running on **Ubuntu/WSL**, create a virtual environment, attach your U2D2 to WSL via `usbipd`, and make sure Dynamixel Wizard is closed or disconnected from the servo before running the code.
- If using a **different Dynamixel model**, update the control table addresses in the code to match your model. These can be found in the official documentation at [emanual.robotis.com](https://emanual.robotis.com).
