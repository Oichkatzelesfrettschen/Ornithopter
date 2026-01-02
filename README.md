# ELEC6212 Biologically Inspired Robotics - Ornithopter Flight Control System

[![CI/CD Pipeline](https://github.com/Oichkatzelesfrettschen/Ornithopter/actions/workflows/ci.yml/badge.svg)](https://github.com/Oichkatzelesfrettschen/Ornithopter/actions)
[![License](https://img.shields.io/badge/license-MIT-blue.svg)](LICENSE)

**Platform**: ESP32 Pico D4  
**Framework**: Arduino with PlatformIO  
**Version**: 2.0 (January 2026)

---

## 📋 Overview

Advanced flight control system for a biologically-inspired ornithopter (flapping wing aircraft) featuring:
- Multi-axis PID stabilization
- Quaternion-based attitude representation
- Formal verification (TLA+ and Z3)
- Modern build system with CI/CD
- Comprehensive R&D documentation

## 🚀 Quick Start

### Prerequisites
- [PlatformIO](https://platformio.org/) IDE or CLI
- Python 3.11+ (for verification tools)
- ESP32 Pico D4 development board
- MPU9250 IMU sensor

### Build and Upload
```bash
# Clone repository
git clone https://github.com/Oichkatzelesfrettschen/Ornithopter.git
cd Ornithopter

# Build firmware
make build

# Upload to device
make upload

# Monitor serial output
make monitor
```

### Using PlatformIO Directly
```bash
# Build release
pio run -e pico32_release

# Upload
pio run -e pico32_release --target upload

# Debug build
pio run -e pico32_debug
```

## 📚 Documentation

Comprehensive research and development documentation available in [`docs/`](docs/):

- **[Comprehensive R&D Report](docs/COMPREHENSIVE_R&D_REPORT.md)** - Complete technical analysis
- **[Technical Debt Analysis](docs/mathematical_foundations/TECHNICAL_DEBT_ANALYSIS.md)** - System gaps and improvements
- **[TLA+ Specification](docs/formal_verification/Ornithopter.tla)** - Formal system specification
- **[Z3 Verification](docs/formal_verification/verify_control.py)** - SMT solver proofs

### Topics Covered
1. **Mathematical Foundations**: Control theory, state-space models, stability analysis
2. **Materials & Fluid Mechanics**: Aeroelasticity, unsteady aerodynamics, wing materials
3. **Rotation Mathematics**: Quaternions, octonions, SLERP, dual quaternions
4. **Machine Learning**: MLP integration, reinforcement learning, adaptive control
5. **Hardware Integration**: Sensor fusion, IMU, environmental sensors, RTOS
6. **Formal Verification**: TLA+ specifications, Z3 proofs, runtime monitoring
7. **Build System**: Modernized PlatformIO, CI/CD, static analysis

## 🏗️ Architecture

```
┌─────────────────────────────────────────┐
│         User Interface (BLE/WiFi)       │
└──────────────┬──────────────────────────┘
               │
┌──────────────▼──────────────────────────┐
│    Control Layer (PID/MLP/Adaptive)     │
└──────────────┬──────────────────────────┘
               │
┌──────────────▼──────────────────────────┐
│   Sensor Fusion (Quaternions/Kalman)    │
│   • IMU (MPU9250)                       │
│   • Pressure, Airspeed                  │
└──────────────┬──────────────────────────┘
               │
┌──────────────▼──────────────────────────┐
│      Actuators (Servos + Motor)         │
└──────────────┬──────────────────────────┘
               │
┌──────────────▼──────────────────────────┐
│   Physical System (Flapping Wings)      │
└─────────────────────────────────────────┘
```

## 🛠️ Development

### Build Targets
```bash
make build          # Build release version
make debug          # Build debug version
make upload         # Upload to device
make monitor        # Serial monitor
make clean          # Clean build artifacts
```

### Code Quality
```bash
make check          # Static analysis (cppcheck, clang-tidy)
make format         # Format code (clang-format)
make size           # Analyze binary size
make memanalyze     # Memory usage analysis
```

### Verification
```bash
make verify         # Run formal verification (Z3)
make docs           # Generate Doxygen documentation
```

## 📊 System Status

| Component | Status | Version |
|-----------|--------|---------|
| PID Control | ✅ Stable | 1.0 |
| Quaternion Math | ✅ Complete | 2.0 |
| IMU Integration | ✅ Working | 1.0 |
| BLE Communication | ✅ Working | 1.0 |
| Formal Verification | ✅ Implemented | 2.0 |
| CI/CD Pipeline | ✅ Active | 2.0 |
| ML Integration | 🟡 Planned | - |
| MPC Controller | 🟡 Research | - |

## 🧪 Testing

### Hardware-in-the-Loop
```bash
# Run with hardware connected
pio test -e pico32_release
```

### Formal Verification
```bash
# Install dependencies
pip install z3-solver

# Run verification suite
python3 docs/formal_verification/verify_control.py
```

## 📝 Project Structure

```
Ornithopter/
├── src/                    # Source code
│   ├── main.cpp           # Main entry point
│   ├── control.cpp        # PID control implementation
│   ├── imu.cpp            # IMU sensor interface
│   ├── quaternion_math.cpp # Quaternion library
│   └── ...
├── include/               # Header files
│   └── quaternion_math.h  # Quaternion mathematics
├── docs/                  # Documentation
│   ├── COMPREHENSIVE_R&D_REPORT.md
│   ├── mathematical_foundations/
│   ├── formal_verification/
│   └── ...
├── .github/workflows/     # CI/CD pipelines
├── platformio.ini         # PlatformIO configuration
├── Makefile              # Build automation
└── README.md             # This file
```

## 🔬 Research & Development

Current R&D focus areas:
- Advanced control (LQR, MPC, adaptive)
- Machine learning integration (MLP, RL)
- Extended sensor suite (pressure, airspeed)
- Real-time OS integration (FreeRTOS)
- Energy-optimal trajectories
- Formation flight algorithms

See [docs/COMPREHENSIVE_R&D_REPORT.md](docs/COMPREHENSIVE_R&D_REPORT.md) for details.

## 🤝 Contributing

### Workflow
1. Create a feature branch
2. Make changes with clear commits
3. Run verification: `make verify`
4. Run tests and static analysis: `make check`
5. Submit pull request
6. CI/CD pipeline validates changes

### Commit Guidelines
```
type(scope): brief description

- Detailed explanation
- Reference issues if applicable
- Date: YYYY-MM-DD
- Author: Your Name
```

Example:
```
feat(control): add LQR controller

- Implement Linear Quadratic Regulator
- Add Riccati equation solver
- Integrate with existing PID system
- Date: 2026-01-02
- Author: John Doe
```

## ⚠️ Note

- Update code before starting work
- Work on your branch and use merge requests
- Write comments with date, name, and description
- Ensure no compile errors before pushing
- Update TODO list when working on modules

## 📋 TODO List

### Control
- [x] PID class
- [x] Balance control algorithm
- [ ] LQR implementation
- [ ] MPC controller
- [ ] Adaptive gains

### Sensors
- [x] IMU (MPU9250) integration
- [x] Quaternion-based orientation
- [ ] Barometric altimeter
- [ ] Airspeed sensor
- [ ] GPS integration

### Actuators
- [x] Servo control
- [x] Brushless motor (ESC)
- [ ] Dynamic mixing
- [ ] Flapping frequency optimization

### Communication
- [x] BLE setup
- [x] UDP debug data
- [ ] Telemetry logging
- [ ] Ground station interface

### Verification
- [x] TLA+ specification
- [x] Z3 SMT verification
- [ ] Runtime monitoring
- [ ] Hardware-in-loop testing

## 📖 References

### Academic Papers
- Shyy et al. (2013) - Aerodynamics of Low Reynolds Number Flyers
- Sane (2003) - The aerodynamics of insect flight

### Textbooks
- Khalil - Nonlinear Systems
- Åström & Murray - Feedback Systems
- Lamport - Specifying Systems (TLA+)

### Standards
- IEEE/ACM Guidelines for Flight Control Software
- DO-178C (Software Considerations in Airborne Systems)

## 📧 Contact

- **Project**: ELEC6212 Biologically Inspired Robotics
- **Institution**: [University Name]
- **Repository**: https://github.com/Oichkatzelesfrettschen/Ornithopter

## 📄 License

This project is licensed under the MIT License - see LICENSE file for details.

---

**Last Updated**: January 2026  
**Version**: 2.0  
**Status**: Active Development