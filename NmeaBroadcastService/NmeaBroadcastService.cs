using System.IO.Ports;
using System.Net;
using System.Net.NetworkInformation;
using System.Net.Sockets;
using System.Text;
using Microsoft.Extensions.Configuration;
using Microsoft.Extensions.Hosting;
using Microsoft.Extensions.Logging;

namespace NmeaBroadcastService
{
    public class NmeaBroadcastService : IHostedService
    {
        private readonly ILogger<NmeaBroadcastService> _logger;
        private readonly IConfiguration _configuration;
        private SerialPort _serialPort;
        private UdpClient _udpClient;
        private CancellationTokenSource _cancellationTokenSource;
        private Task _readTask;
        private readonly StringBuilder _nmeaBuffer;
        private string _broadcastIp;
        private int _udpPort;

        public NmeaBroadcastService(
            ILogger<NmeaBroadcastService> logger,
            IConfiguration configuration
        )
        {
            _logger = logger;
            _configuration = configuration;
            _nmeaBuffer = new StringBuilder();
        }

        public Task StartAsync(CancellationToken cancellationToken)
        {
            _logger.LogInformation("Starting NMEA Service...");

            // Load configuration from appsettings.json
            string comPort = _configuration["ComPort"];
            int baudRate = int.Parse(_configuration["BaudRate"]);
            string parityString = _configuration["Parity"];
            Parity parity = (Parity)Enum.Parse(typeof(Parity), parityString, true);
            _broadcastIp = _configuration["BroadcastIP"];
            _udpPort = int.Parse(_configuration["BroadcastPort"]);

            // Log configuration for verification
            _logger.LogInformation(
                "Loaded Configuration: COM Port={ComPort}, BaudRate={BaudRate}, Parity={Parity}, BroadcastIP={BroadcastIP}, BroadcastPort={BroadcastPort}",
                comPort,
                baudRate,
                parity,
                _broadcastIp,
                _udpPort
            );

            // Verify if the COM port exists
            if (!SerialPort.GetPortNames().Contains(comPort))
            {
                _logger.LogError("COM port {comPort} does not exist or is unavailable.", comPort);
                return Task.CompletedTask;
            }

            // Validate the broadcast address
            if (!IsValidBroadcastIP(_broadcastIp))
            {
                _logger.LogError(
                    "The IP address {_broadcastIp} is not valid or not allowed on this system.",
                    _broadcastIp
                );
                return Task.CompletedTask;
            }

            // Set up serial port
            _serialPort = new SerialPort(comPort, baudRate, parity, 8, StopBits.One);
            _serialPort.DataReceived += DataReceivedHandler;
            _serialPort.Open();

            // Set up UDP client
            _udpClient = new UdpClient { EnableBroadcast = true };

            // Start the background task with a cancellation token
            _cancellationTokenSource = CancellationTokenSource.CreateLinkedTokenSource(
                cancellationToken
            );
            _readTask = Task.Run(
                () => Read(_cancellationTokenSource.Token),
                _cancellationTokenSource.Token
            );

            _logger.LogInformation("NMEA Service started successfully.");
            return Task.CompletedTask;
        }

        public async Task StopAsync(CancellationToken cancellationToken)
        {
            _logger.LogInformation("Stopping NMEA Service...");

            // Signal cancellation
            _cancellationTokenSource.Cancel();

            // Wait for the read task to finish
            if (_readTask != null)
            {
                try
                {
                    await Task.WhenAny(_readTask, Task.Delay(Timeout.Infinite, cancellationToken));
                }
                catch (TaskCanceledException)
                {
                    _logger.LogWarning("NMEA Service stop was forcefully canceled.");
                }
            }

            _serialPort?.Close();
            _udpClient?.Close();

            _logger.LogInformation("NMEA Service stopped.");
        }

        private async Task Read(CancellationToken cancellationToken)
        {
            try
            {
                while (!cancellationToken.IsCancellationRequested)
                {
                    await Task.Delay(100, cancellationToken); // Check cancellation token every 100ms
                    // Additional logic for handling serial port data
                }
            }
            catch (OperationCanceledException)
            {
                _logger.LogInformation("Cancellation requested, stopping read loop.");
            }
            catch (Exception ex)
            {
                _logger.LogError(ex, "Error in read loop.");
            }
        }

        private void DataReceivedHandler(object sender, SerialDataReceivedEventArgs e)
        {
            try
            {
                string inData = _serialPort.ReadExisting(); // Read the data received
                _nmeaBuffer.Append(inData); // Append to buffer

                while (_nmeaBuffer.ToString().Contains("\r\n"))
                {
                    string nmeaSentence = ExtractCompleteSentence();
                    if (!string.IsNullOrEmpty(nmeaSentence))
                    {
                        SendUdpBroadcast(nmeaSentence); // Send complete sentence
                    }
                }
            }
            catch (System.Exception ex)
            {
                _logger.LogError(ex, "Error in DataReceivedHandler.");
            }
        }

        private string ExtractCompleteSentence()
        {
            string bufferString = _nmeaBuffer.ToString();
            int start = bufferString.IndexOf('$');
            int end = bufferString.IndexOf("\r\n");

            if (start != -1 && end != -1 && end > start)
            {
                string sentence = bufferString.Substring(start, (end - start + 2)); // Include '\r\n'
                _nmeaBuffer.Remove(0, end + 2);
                return sentence;
            }
            return null;
        }

        private void SendUdpBroadcast(string message)
        {
            try
            {
                IPEndPoint ipEndPoint = new(IPAddress.Parse(_broadcastIp), _udpPort);
                byte[] data = Encoding.ASCII.GetBytes(message);
                _udpClient.Send(data, data.Length, ipEndPoint);
            }
            catch (System.Exception ex)
            {
                _logger.LogError(ex, "Error sending UDP broadcast");
            }
        }

        private bool IsValidBroadcastIP(string ipAddress)
        {
            try
            {
                var broadcastIp = IPAddress.Parse(ipAddress);
                foreach (var networkInterface in NetworkInterface.GetAllNetworkInterfaces())
                {
                    foreach (
                        var unicastAddress in networkInterface.GetIPProperties().UnicastAddresses
                    )
                    {
                        var localIp = unicastAddress.Address;
                        var subnetMask = unicastAddress.IPv4Mask;

                        if (
                            localIp.AddressFamily == AddressFamily.InterNetwork
                            && subnetMask != null
                        )
                        {
                            var broadcastAddress = GetBroadcastAddress(localIp, subnetMask);
                            if (broadcastIp.Equals(broadcastAddress))
                            {
                                return true;
                            }
                        }
                    }
                }
            }
            catch (System.Exception ex)
            {
                _logger.LogError(ex, "Error in IP validation.");
            }
            return false;
        }

        private static IPAddress GetBroadcastAddress(IPAddress address, IPAddress subnetMask)
        {
            byte[] ipAddressBytes = address.GetAddressBytes();
            byte[] subnetMaskBytes = subnetMask.GetAddressBytes();

            if (ipAddressBytes.Length != subnetMaskBytes.Length)
            {
                throw new ArgumentException("Length of IP address and subnet mask do not match.");
            }

            byte[] broadcastAddress = new byte[ipAddressBytes.Length];
            for (int i = 0; i < broadcastAddress.Length; i++)
            {
                broadcastAddress[i] = (byte)(ipAddressBytes[i] | (subnetMaskBytes[i] ^ 255));
            }

            return new IPAddress(broadcastAddress);
        }
    }
}
