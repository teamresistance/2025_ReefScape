# FRC Robot Code Guide

## Build Commands
- Build: `./gradlew build`
- Deploy to robot: `./gradlew deploy`
- Run tests: `./gradlew test`
- Format code: `./gradlew spotlessApply`
- Replay logs: `./gradlew replayWatch`

## Code Style Guidelines
- **Formatting**: Google Java Format (enforced by Spotless)
- **Naming**:
  - Classes: PascalCase (`DriveSubsystem`)
  - Methods: camelCase (`configureButtonBindings()`)
  - Constants: `kConstantName` with 'k' prefix
  - Member variables: `m_variableName` with 'm_' prefix
  - Static/final fields: SCREAMING_SNAKE_CASE
- **Structure**: Command-based robot design with subsystems
- **Imports**: Standard Java conventions, unused imports removed
- **Error Handling**: Use Alert system for critical errors
- **Tools**: AdvantageKit for logging, PathPlanner for autonomous
