# NmeaBroadcastService

## Overview

The **NmeaBroadcastService** is a Windows service written in .NET 8.0 that reads data from a serial port (NMEA protocol) and broadcasts the data over a network using UDP. The service is self-contained, providing serial communication over a specified COM port and sending UDP broadcasts to a configurable IP address and port.

## Features

- Reads NMEA data from a specified COM port.
- Broadcasts NMEA sentences over a UDP network.
- Configurable through `appsettings.json` for COM port, baud rate, parity, and network settings.
- Graceful startup and shutdown with proper cancellation handling.

## Prerequisites

- **.NET 8.0 SDK** installed on the machine.
- Serial port hardware available (or virtual COM port).
- A valid broadcast IP address and port for network communication.

## Installation

1. Clone this repository:

   ```bash
   git clone http://192.168.99.71:10037/ETS/NmeaBroadcastService.git
   ```

2. Navigate to the project directory:

   ```bash
   cd NmeaBroadcastService
   ```

3. Build the project:

   ```bash
   dotnet build
   ```

4. Publish the service as a self-contained executable:

   ```bash
   dotnet publish -c Release -r win-x64 --self-contained
   ```

5. Install the service using the Windows Service Control Manager (sc.exe) or by running the MSI installer (if available).

## Running the Service

To start the service, use the Windows Service Manager or the command line:

   ```bash
   sc start NmeaBroadcastService
   ```

To stop the service:

   ```bash
   sc stop NmeaBroadcastService
   ```

## Configuration

The service's configuration is managed via the `appsettings.json` file located in the root of the project or published output directory. The file contains settings for the serial communication and UDP broadcast parameters.

### Example `appsettings.json`

```json
{
  "Logging": {
    "LogLevel": {
      "Default": "Information",
      "Microsoft": "Warning",
      "System": "Warning"
    }
  },
  "ComPort": "COM3",
  "BaudRate": "4800",
  "Parity": "None",
  "BroadcastIP": "192.168.2.255",
  "BroadcastPort": "8888"
}
```

### Configurable Parameters:

- **ComPort**: Specifies the COM port for the serial connection (e.g., "COM3").
- **BaudRate**: Defines the baud rate for the serial connection (e.g., 4800).
- **Parity**: Sets the parity for the serial communication. Options are: `None`, `Odd`, `Even`.
- **BroadcastIP**: The broadcast IP address to which the NMEA data will be sent.
- **BroadcastPort**: The UDP port number for broadcasting the data.

### How to Edit Configuration

1. Open `appsettings.json` in a text editor.
2. Modify the parameters as needed.
   - **ComPort**: Change to the appropriate COM port name (e.g., `"COM4"`).
   - **BaudRate**: Set the correct baud rate for your device (e.g., `9600`).
   - **Parity**: Adjust the parity setting (e.g., `"Odd"`).
   - **BroadcastIP**: Set the IP address for broadcasting.
   - **BroadcastPort**: Define the UDP port number for broadcasting.
3. Save the file and restart the service to apply the changes.

## Logging

The service logs its activities, including startup, shutdown, data receipt, and errors. Logs are written based on the configuration in `appsettings.json`. By default, it logs at the `Information` level.

To adjust the logging level, modify the `LogLevel` settings in the `Logging` section of `appsettings.json`:

```json
"Logging": {
  "LogLevel": {
    "Default": "Debug"
  }
}
```

## License

This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for details.

## Contributing

Feel free to open issues or submit pull requests if you would like to contribute to this project.
