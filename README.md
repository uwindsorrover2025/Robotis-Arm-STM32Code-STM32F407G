# STM32F407 Robotic Arm Controller

A sophisticated 6-DOF robotic arm control system built on the STM32F407 microcontroller, featuring real-time encoder feedback, multi-motor control, and CAN bus communication.

## Overview

This project implements a complete control system for a 6-axis robotic arm with the following features:
- Real-time position feedback from 5 quadrature encoders
- Control of 6 different motor types (NEO, Grimson, and stepper motors)
- CAN bus interface for remote control and monitoring
- Multiple control modes: Position (PID), Velocity, and Manual
- Automatic homing using encoder index pulses
- Emergency stop functionality
- Comprehensive status reporting

## Hardware Configuration

### Microcontroller
- **MCU**: STM32F407VGT6
- **Clock**: 168 MHz (External 8MHz crystal with PLL)
- **Development Board**: STM32F4-Discovery or compatible

### Motor Configuration

| Joint | Motor Type | Control Method | Timer/Channel | GPIO Pins |
|-------|-----------|----------------|---------------|-----------|
| Base | NEO Motor | Spark Max PWM (50Hz) | TIM5_CH1 | PA0 |
| Shoulder | NEO Motor | Spark Max PWM (50Hz) | TIM5_CH3 | PA2 |
| Elbow | NEO Motor | Spark Max PWM (50Hz) | TIM5_CH4 | PA3 |
| Wrist1 | Grimson Motor | H-Bridge PWM (20kHz) | TIM9_CH1 | PE5 (PWM), PC5 (DIR) |
| Wrist2 | Grimson Motor | H-Bridge PWM (20kHz) | TIM9_CH2 | PE6 (PWM), PB0 (DIR) |
| End Effector | Stepper Motor | Step/Dir | TIM11_CH1 | PB9 (STEP), PB2 (DIR), PB1 (EN) |

### Encoder Configuration

| Joint | Timer | Encoder Pins | Index Pin | Resolution |
|-------|-------|--------------|-----------|------------|
| Base | TIM1 | PE9/PE11 | PC0 | Quadrature |
| Shoulder | TIM2 | PA15/PB3 | PC1 | Quadrature |
| Elbow | TIM3 | PA6/PA7 | PC2 | Quadrature |
| Wrist1 | TIM4 | PD12/PD13 | - | Quadrature |
| Wrist2 | TIM8 | PC6/PC7 | - | Quadrature |

### Communication
- **CAN Bus**: CAN1 on PD0/PD1 (500 kbps)
- **Debug LEDs**: LD3 (Orange - Heartbeat), LD5 (Red - Error)

## Software Architecture

### Control Loop Timing
- **Motor Update Rate**: 1 kHz (1ms)
- **Encoder Data Transmission**: 50 Hz (20ms)
- **Motor Status Transmission**: 10 Hz (100ms)
- **System Status**: 2 Hz (500ms)
- **Debug Info**: 1 Hz (1000ms)

### Motor Control Modes
1. **IDLE**: Motor disabled
2. **VELOCITY**: Direct velocity control (-1000 to +1000)
3. **POSITION**: PID position control with configurable gains
4. **MANUAL**: Direct PWM control via CAN commands

### PID Control
Each motor has configurable PID parameters:
- Default gains: Kp=2.0, Ki=0.1, Kd=0.5 (NEO motors)
- Anti-windup implementation
- Position limits and velocity limits per motor

## CAN Protocol

### Transmitted Messages (From STM32)

| CAN ID | Description | Data Format |
|--------|-------------|-------------|
| 0x100-0x104 | Encoder Data | [position(4B)][velocity(4B)] |
| 0x200 | System Status | [encoder_states(5B)][heartbeat][index_count][marker] |
| 0x201 | Debug Info | [timer_value(2B)][index_states][homing_info] |
| 0x210 | Motor Status | [motor_states(6B)][emergency_stop][marker] |

### Received Commands (To STM32)

| CAN ID | Command | Data Format |
|--------|---------|-------------|
| 0x301 | Start Homing | [encoder_id][reserved...] |
| 0x302 | Reset Encoder | [encoder_id][reserved...] |
| 0x303 | Enable/Disable Encoder | [encoder_id][enable][reserved...] |
| 0x310 | Set Motor Velocity | [motor_id][velocity_high][velocity_low][reserved...] |
| 0x311 | Set Motor Position | [motor_id][pos_byte3][pos_byte2][pos_byte1][pos_byte0][reserved...] |
| 0x312 | Emergency Stop | - |
| 0x313 | Enable/Disable Motor | [motor_id][enable][reserved...] |
| 0x314 | Set PID Gains | [motor_id][kp×10][ki×100][kd×10][reserved...] |
| 0x315 | Coordinated Movement | [base_pos(2B)][shoulder_pos(2B)][elbow_pos(2B)][reserved...] |
| 0x316 | Stepper Move | [steps(2B)][frequency(2B)][reserved...] |
| 0x317 | Clear Emergency Stop | - |
| 0x318 | Manual Motor Control | [motor_id][output_high][output_low][reserved...] |
| 0x319 | Set Position Limits | [motor_id][min(2B)][max(2B)][reserved...] |
| 0x320 | Home All Axes | - |

## Building and Flashing

### Prerequisites
- STM32CubeIDE 1.12.0 or later
- ST-Link V2 programmer or integrated debugger
- ARM GCC toolchain (included with STM32CubeIDE)

### Build Instructions
1. Clone the repository:
   ```bash
   git clone https://github.com/uwindsorrover2025/Robotis-Arm-STM32Code-STM32F407G.git
   cd Robotis-Arm-STM32Code-STM32F407G
   ```

2. Open STM32CubeIDE and import the project:
   - File → Import → General → Existing Projects into Workspace
   - Select the cloned directory
   - Click Finish

3. Build the project:
   - Project → Build Project (or press Ctrl+B)

4. Flash to the microcontroller:
   - Run → Debug (or press F11)
   - The debugger will flash the program and halt at main()
   - Press F8 to run

### Debug Configuration
The project includes debug outputs via CAN messages. Monitor CAN ID 0x201 for detailed debug information about encoder states and homing status.

## Safety Features

1. **Emergency Stop**: CAN command 0x312 immediately stops all motors
2. **Position Limits**: Each motor has configurable min/max position limits
3. **Velocity Limits**: Maximum velocity is limited per motor type
4. **Watchdog Timer**: System monitoring (ready to enable)
5. **Error LED**: Red LED (LD5) indicates system errors

## Usage Example

### Basic Motor Control Sequence
```python
# Example Python code for CAN control
import can

bus = can.interface.Bus(channel='can0', bustype='socketcan', bitrate=500000)

# Enable motor 0 (Base)
msg = can.Message(arbitration_id=0x313, data=[0, 1, 0, 0, 0, 0, 0, 0])
bus.send(msg)

# Set velocity mode and move at 50% speed
msg = can.Message(arbitration_id=0x310, data=[0, 0x01, 0xF4, 0, 0, 0, 0, 0])  # 500/1000 = 50%
bus.send(msg)

# Stop motor
msg = can.Message(arbitration_id=0x310, data=[0, 0, 0, 0, 0, 0, 0, 0])
bus.send(msg)
```

### Homing Sequence
```python
# Start homing for base encoder
msg = can.Message(arbitration_id=0x301, data=[0, 0, 0, 0, 0, 0, 0, 0])
bus.send(msg)

# Monitor status messages (0x200) for homing completion
```

## Troubleshooting

### Common Issues

1. **Motors not responding**
   - Check emergency stop status (clear with 0x317)
   - Verify motor is enabled (use command 0x313)
   - Check CAN connection and termination resistors

2. **Encoder counts erratic**
   - Verify encoder wiring and shielding
   - Check for proper pull-up resistors on encoder lines
   - Reduce counting frequency if needed

3. **CAN communication errors**
   - Ensure 120Ω termination resistors at both ends
   - Verify baud rate matches (500 kbps default)
   - Check CAN transceiver power supply

### Debug Commands
Special debug commands are available for Motor 0 via CAN ID 0x360:
- 0x01: Test GPIO toggle
- 0x02: Send motor 0 debug info
- 0x03: Verify TIM5 configuration
- 0x04: Force PWM output
- 0x05: Force neutral position

## Project Structure

```
arm_encoders_pwm_afterclock/
├── Core/
│   ├── Inc/           # Header files
│   ├── Src/           # Source files
│   │   └── main.c     # Main application code
│   └── Startup/       # Startup assembly code
├── Drivers/           # STM32 HAL and CMSIS drivers
├── Debug/             # Build output (excluded from git)
├── .gitignore         # Git ignore file
├── arm_encoders_pwm_afterclock.ioc  # STM32CubeMX project file
└── README.md          # This file
```

## Contributing

When contributing to this project:
1. Follow the existing code style
2. Test all changes with actual hardware
3. Update documentation for new features
4. Submit pull requests with clear descriptions

## License

This project is part of the University of Windsor Rover Team's robotics development. Please contact the team for licensing information.

## Contact

University of Windsor Rover Team 2025
- GitHub: [uwindsorrover2025](https://github.com/uwindsorrover2025)
- Project: [Robotis-Arm-STM32Code-STM32F407G](https://github.com/uwindsorrover2025/Robotis-Arm-STM32Code-STM32F407G)

## Acknowledgments

- STMicroelectronics for the STM32 HAL libraries
- The University of Windsor Rover Team
- Contributors to the STM32 development ecosystem