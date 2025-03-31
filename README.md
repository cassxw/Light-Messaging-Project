# Light-Based Messaging System with STM Development Boards

## Overview
This project explores an innovative approach to data communication using **light signals**. By leveraging **STM development boards**, we demonstrate a seamless **embedded system-to-embedded system** messaging system. The core objective is to transmit **analog data**—specifically, readings from an onboard potentiometer (POT) sampled via an **ADC**—from a **Transmitter (Sensor Node - SN)** to a **Receiver (Central Node - CN)**.

## Key Features
- **Push-button-triggered data sampling**
- **GPIO-based data transmission** using the ‘Bit-Bang’ method
- **Custom Light-of-Things (LoT) Message Protocol** with parity checks
- **Real-time error detection and correction mechanisms**
- **Continuous listening mode at the receiver** for improved responsiveness
- **LCD display for real-time decoded data output**

## System Architecture
The system consists of:
- **Transmitter (Sensor Node - SN)**
  - Equipped with an **ADC** to sample an onboard **POT**
  - Transmits the sampled data as a **binary message** using **light pulses** (LED)
  - Implements the **LoT Message Protocol** for structured message transmission
  - Includes a **checkpoint mechanism** to track transmitted samples

- **Receiver (Central Node - CN)**
  - Continuously listens for incoming **light-based binary messages**
  - Decodes the received data and displays it on an **LCD**
  - Implements **error detection** via **parity checks**
  - Compares received samples with checkpoint messages to ensure data integrity

## Light-of-Things (LoT) Message Protocol
The LoT protocol defines two types of messages:
### **1. Data Message**
Transmits sampled ADC values from the Transmitter to the Receiver.
- **Start-of-Text (SOT) [1-bit]** - LED ON for 1s
- **Message Identifier [1-bit]** - LED OFF for 1s (Data Message)
- **Data Packet [16-bit]** - Represents ADC value; LED ON (1) / OFF (0) for 500ms
- **Even Parity Bit [1-bit]** - Used for error detection
- **End-of-Text (EOT) [1-bit]** - LED blinks rapidly for 1s

### **2. Checkpoint Message**
Ensures message reliability by tracking sent/received samples.
- **Start-of-Text (SOT) [1-bit]** - LED ON for 1s
- **Message Identifier [1-bit]** - LED ON for 1s (Checkpoint Message)
- **Checkpoint Packet [16-bit]** - Represents sample count
- **Even Parity Bit [1-bit]** - Ensures data integrity
- **End-of-Text (EOT) [1-bit]** - LED blinks rapidly for 1s

## Error Detection & Handling
1. **Parity Bit Check** - Detects single-bit errors within data packets
2. **Checkpoint Message Verification** - Compares expected vs. received samples
   - If **mismatch** → LED blinks rapidly for 2s (Error Alert)
   - If **match** → LED remains ON for 2s (Success)

## Design Choices & Modifications
- **GPIO ‘Bit-Bang’ Method:** Direct GPIO manipulation for **binary serial transmission** over light signals.
- **Continuous Listening Mode:** The Receiver remains in an **always-on** state for dynamic responsiveness.
- **Parity Bit Addition:** Enhances error detection beyond just missing messages.
- **LCD Display Output:** Provides real-time data visualisation.

## Project Benefits
✅ **Real-time data exchange** between embedded systems.
✅ **Error detection & correction** ensures reliability.
✅ **Simple yet effective** light-based communication.
✅ **Adaptable** for various IoT and sensor-based applications.

## Future Improvements
- Implement **Light-Dependent Resistors (LDRs)** for more sophisticated optical reception.
- Improve **timing accuracy** with hardware-based PWM signals.
- Enhance **data rate and efficiency** by optimising transmission protocols.

---
