using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Diagnostics;
using System.IO;
using System.IO.Pipes;
using System.Linq;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using System.Runtime.InteropServices;

// See http://stackoverflow.com/questions/9061655/redirect-cout-from-c-dll-to-a-textbox-in-c-sharp

namespace UIWindow
{
    public class ConsoleRedirector : IDisposable
    {
        private static ConsoleRedirector _instance;

        public static void attach(ProgressChangedEventHandler handler, bool forceConsoleRedirection)
        {
            Debug.Assert(null == _instance);
            _instance = new ConsoleRedirector(handler, forceConsoleRedirection);

        }

        public static void detatch()
        {
            _instance.Dispose();
            _instance = null;
        }

        public static bool isAttached
        {
            get
            {
                return null != _instance;
            }
        }

        private static void ResetConsoleOutStream()
        {
            //Force console to recreate its output stream the next time Write/WriteLine is called
            typeof(Console).GetField("_out", System.Reflection.BindingFlags.Static | System.Reflection.BindingFlags.NonPublic).SetValue(null, null);
        }

        private const int PERIOD = 500;
        private const int BUFFER_SIZE = 4096;
        private volatile bool _isDisposed;
        private BackgroundWorker _worker;
        private readonly IntPtr _stdout;
        private readonly Mutex _sync;
        private readonly System.Threading.Timer _timer;
        private readonly char[] _buffer;
        private readonly AnonymousPipeServerStream _outServer;
        private readonly TextReader _outClient;
        private readonly bool _forceConsoleRedirection;

        //private StreamWriter _consoleStandardOut;

        private ConsoleRedirector(ProgressChangedEventHandler handler, bool forceConsoleRedirection)
        {
            bool ret;
            _forceConsoleRedirection = forceConsoleRedirection;

            if (!_forceConsoleRedirection)
            {
                //Make sure Console._out is initialized before we redirect stdout, so the redirection won't affect it
                TextWriter temp = Console.Out;
            }

            AnonymousPipeClientStream client;

            _worker = new BackgroundWorker();
            _worker.ProgressChanged += handler;
            _worker.DoWork += _worker_DoWork;
            _worker.WorkerReportsProgress = true;

            _stdout = GetStdHandle(STD_OUTPUT_HANDLE);

            _sync = new Mutex();
            _buffer = new char[BUFFER_SIZE];

            _outServer = new AnonymousPipeServerStream(PipeDirection.Out);
            client = new AnonymousPipeClientStream(PipeDirection.In, _outServer.ClientSafePipeHandle);
            Debug.Assert(_outServer.IsConnected);
            _outClient = new StreamReader(client, Encoding.Default);
            ret = SetStdHandle(STD_OUTPUT_HANDLE, _outServer.SafePipeHandle.DangerousGetHandle());
            Debug.Assert(ret);

            if (_forceConsoleRedirection)
            {
                ResetConsoleOutStream(); //calls to Console.Write/WriteLine will now get made against the redirected stream
            }

            _worker.RunWorkerAsync(_outClient);

            _timer = new System.Threading.Timer(flush, null, PERIOD, PERIOD);

        }

        void _worker_DoWork(object sender, DoWorkEventArgs e)
        {
            BackgroundWorker worker = (BackgroundWorker)sender;
            TextReader client = (TextReader)e.Argument;
            try
            {
                while (true)
                {
                    int read = client.Read(_buffer, 0, BUFFER_SIZE);
                    if (read > 0)
                        worker.ReportProgress(0, new string(_buffer, 0, read));
                }
            }
            catch (ObjectDisposedException)
            {
                // Pipe was closed... terminate

            }
            catch (Exception ex)
            {
                throw ex;
            }
        }

        private void flush(object state)
        {
            _outServer.Flush();
        }

        public void Dispose()
        {
            Dispose(true);
        }

        ~ConsoleRedirector()
        {
            Dispose(false);
        }

        private void Dispose(bool disposing)
        {
            if (!_isDisposed)
            {
                lock (_sync)
                {
                    if (!_isDisposed)
                    {
                        _isDisposed = true;
                        _timer.Change(Timeout.Infinite, Timeout.Infinite);
                        _timer.Dispose();
                        flush(null);

                        try { SetStdHandle(STD_OUTPUT_HANDLE, _stdout); }
                        catch (Exception) { }
                        _outClient.Dispose();
                        _outServer.Dispose();

                        if (_forceConsoleRedirection)
                        {
                            ResetConsoleOutStream(); //Calls to Console.Write/WriteLine will now get redirected to the original stdout stream
                        }

                    }
                }
            }
        }

        private const int STD_OUTPUT_HANDLE = -11;

        [DllImport("kernel32.dll")]
        [return: MarshalAs(UnmanagedType.Bool)]
        private static extern bool SetStdHandle(int nStdHandle, IntPtr hHandle);

        [DllImport("kernel32.dll")]
        private static extern IntPtr GetStdHandle(int nStdHandle);
    }
}