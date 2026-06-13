using System.Windows;
using System.Windows.Threading;

namespace KinectScanner
{
    public partial class App : Application
    {
        private void App_DispatcherUnhandledException(object sender, DispatcherUnhandledExceptionEventArgs e)
        {
            MessageBox.Show(
                "Unexpected error:\n\n" + e.Exception.Message,
                "Kinect 3D Scanner",
                MessageBoxButton.OK,
                MessageBoxImage.Error);
            e.Handled = true;
        }
    }
}
