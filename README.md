# Smart CAN Bus Switcher

This project is a DIY home automation solution for smart lighting, built around a custom PCB that fits into a standard electrical switch box. It uses a wired CAN bus for robust communication, controlled by a central single-board computer like Orange Pi. This approach provides industrial-grade reliability for your smart home.

## Key Features

*   **Dual Load Control:** Each module has two on-board triacs, allowing it to control two independent lighting groups.
*   **Dimming Functionality:** Supports dimmable LED lamps for smooth brightness adjustments, enabling "sunrise" or "soft evening light" scenes.
*   **Built-in Climate Monitoring:** An integrated temperature sensor in every switch turns your entire home into a climate monitoring network.
*   **Reliable Communication:** Operates over a 2-pair cable, with one pair for 12/24V power and the other for the CAN differential line.

## System Architecture

The heart of the system is a single-board computer (e.g., Orange Pi), which acts as the central brain.

1.  **Central Controller (Orange Pi):**
    *   Collects data from all switches via the CAN bus.
    *   Processes all automation logic (e.g., an "All Off" button at the entrance).
    *   Integrates with external services and voice assistants like Yandex Alice.
2.  **Smart Switch Modules:** Custom PCBs installed in wall switch boxes. They execute commands received from the controller and report status and sensor data.
3.  **CAN Bus:** The backbone of the system, providing a high-speed, reliable, and noise-immune communication link between the controller and all modules.

The server software can be implemented using platforms like **Home Assistant** or custom applications written in **Python/C++**, allowing for easy integration with smart home ecosystems. A command like *"Alice, set the living room light to 30%"* is sent to the Orange Pi, which instantly relays it to the appropriate module on the CAN bus to adjust the light level.

## Why CAN Bus over Zigbee/Wi-Fi?

For a new construction or major renovation, a wired solution offers significant advantages over popular wireless protocols.

| Characteristic      | This Solution (CAN Bus)                                   | Popular Wireless (Zigbee)                                |
| ------------------- | --------------------------------------------------------- | -------------------------------------------------------- |
| **Reliability**     | Maximum. Immune to interference from Wi-Fi or microwaves. | Depends on RF signal quality and the number of repeaters. |
| **Speed**           | Instantaneous response (milliseconds).                    | Delays are possible as nodes wake up or the mesh network adjusts. |
| **Power**           | Centralized. No batteries to replace in sensors.          | Requires battery changes or modules connected to a neutral wire. |
| **Scalability**     | Line length can extend to hundreds of meters.             | Limited by the radio signal range.                       |

The main benefit: a wired system is a "set it and forget it" solution. It won't fail because of a router firmware update or a new neighbor's Wi-Fi network.

## Project Structure

*   **/firmware/**: Contains firmware for the microcontroller (STM32/Arduino) on the switch modules. Includes projects for PlatformIO and STM32Cube.
*   **/hardware/**: Datasheets for components used in the project.
*   **/production/**: PCB manufacturing files, including KiCad project, Gerber files, and Bill of Materials (BOM).
*   **/software/**: Server-side applications and scripts for the Orange Pi.
*   **/documents/**: Design documents, manuals, and reports.

## Installation and Wiring

The system uses a 2-pair cable (e.g., KSPV or shielded twisted pair) for both power and data.

*   **Pair 1:** Power (GND / VCC)
*   **Pair 2:** Data (CAN-High / CAN-Low)

The PCB is designed to be compact, leaving enough space for wiring even in shallow switch boxes.

## Future Plans

The next article will provide a detailed breakdown of the PCB schematic, component selection, and the board layout process in EasyEDA/KiCad.