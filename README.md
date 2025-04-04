<h1 align="left">
FRC 2025 Robot Code
<img src="https://waterga.me/share/logo.svg" align="right" width="45" height="45" />
</h1>

Welcome to the official repository for **FRC Team 4533 Phoenix's 2025 Robot Code**! This repository contains all the code for our robot, including subsystems, commands, and autonomous routines.

## Documentation


NOTE THIS VERSION uses static IMU and SWERVELIB dirs, and can't be updated easily.
Please use it for testing and as an example
Purpose for this version is to try to deal with the NAVX imu that somtimes (about 1 in 10) seems to not initialize correctly.
this version tries to do a 50hz refresh rate of imu and do a reset of some form, that studica decides to not do.
This version also introduces a SPI bus reset for the NAVX see the article
https://www.chiefdelphi.com/t/navx2-disconnecting-reconnecting-intermittently-not-browning-out/425487/39?page=2



Comprehensive documentation for this project is available at [FRC 2025 Robot Docs](https://4533-phoenix.github.io/frc-2025-robot/).

## Team Website

Learn more about **FRC Team 4533 Phoenix** by visiting our [official website](https://phoenix4533.org/).

## Getting Started

### Prerequisites

- [WPILib](https://docs.wpilib.org/en/stable/index.html)
- Java 17
- Gradle 8.0 or higher
- A compatible IDE (e.g., VS Code with WPILib extension)

### Cloning the Repository

```bash
git clone https://github.com/4533-Phoenix/frc-2025-robot.git
cd frc-2025-robot
```

### Building and Deploying

1. Open the project in your IDE.
2. Connect to the robot's network.
3. Use the WPILib extension to build and deploy the code.

## Contributing

We welcome contributions from team members and the community! To contribute:

1. Fork the repository.
2. Create a new branch for your feature or bug fix.
3. Ensure all workflows pass successfully before submitting your changes.
4. Document all new code thoroughly, including Javadoc comments for methods, classes, and fields.
5. Submit a pull request with a detailed description of your changes.

Pull requests that do not meet the documentation or workflow requirements will not be accepted.

## License

This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for details.

## Contact

For questions or support, please contact us via our [team website](https://phoenix4533.org/).

---
*Built with ❤️ by FRC Team 4533 Phoenix.*
