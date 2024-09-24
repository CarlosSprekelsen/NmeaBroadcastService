using Microsoft.Extensions.Configuration;
using Microsoft.Extensions.DependencyInjection;
using Microsoft.Extensions.Hosting;

namespace NmeaBroadcastService
{
    internal static class Program
    {
        public static void Main(string[] args)
        {
            CreateHostBuilder(args).Build().Run();
        }

        public static IHostBuilder CreateHostBuilder(string[] args) =>
            Host.CreateDefaultBuilder(args)
                .UseWindowsService() // Make sure to add this for Windows services
                .ConfigureAppConfiguration(
                    (hostingContext, config) =>
                    {
                        config.SetBasePath(AppContext.BaseDirectory); // Ensure proper path is set
                        config.AddJsonFile(
                            "appsettings.json",
                            optional: true,
                            reloadOnChange: true
                        ); // Load appsettings.json
                    }
                )
                .ConfigureServices(
                    (hostContext, services) =>
                    {
                        services.AddHostedService<NmeaBroadcastService>();
                    }
                );
    }
}
