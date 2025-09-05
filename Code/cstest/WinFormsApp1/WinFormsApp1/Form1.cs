using System;
using System.IO.Ports;
using System.Text.RegularExpressions;
using System.Windows.Forms;
using DM_USB2CAN;

namespace WinFormsApp1
{
    public partial class Form1 : Form
    {
        private SerialPort serialPort = new SerialPort();
        private CanProcess dm_can = new CanProcess();
        private System.Windows.Forms.Timer controlTimer = new System.Windows.Forms.Timer();
        private byte[] dataTemp = new byte[4096];

        public float P_MIN = -12.5f, P_MAX = 12.5f;
        public float V_MIN = -45.0f, V_MAX = 45.0f;
        public float KP_MIN = 0.0f, KP_MAX = 500.0f;
        public float KD_MIN = 0.0f, KD_MAX = 5.0f;
        public float T_MIN = -18.0f, T_MAX = 18.0f;

        private int slaveId = 0x06;
        private string canId => (0x000 + slaveId).ToString("X4");

        public Form1()
        {
            InitializeComponent(); // <- this must stay!
            SetupSerial();
            EnableMotor();

            controlTimer.Interval = 10;
            controlTimer.Tick += SendMITFrame;
            controlTimer.Start();
        }

        private void SetupSerial()
        {
            string[] ports = SerialPort.GetPortNames();
            if (ports.Length == 0)
            {
                MessageBox.Show("No COM ports found!");
                return;
            }

            string portName = ports[0];
            serialPort.PortName = portName;
            serialPort.PortName = "COM4"; // <- replace with actual port name

            serialPort.BaudRate = 921600;
            serialPort.DataBits = 8;
            serialPort.StopBits = StopBits.One;
            serialPort.Parity = Parity.None;
            serialPort.ReadTimeout = 500;
            serialPort.WriteTimeout = 500;

            serialPort.DataReceived += (sender, e) =>
            {
                int length = serialPort.BytesToRead;
                serialPort.Read(dataTemp, 0, length);
                string hexString = BitConverter.ToString(dataTemp, 0, length).Replace("-", "");
                Console.WriteLine("Receive: " + hexString);
            };

            serialPort.Open();
        }

        private void EnableMotor()
        {
            byte[] enableMsg = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC };
            dm_can.SendToCanData(serialPort, enableMsg, slaveId.ToString("X4"), 1, 1, true, 8);
            Console.WriteLine("✅ Motor enabled");
        }

        private void SendMITFrame(object sender, EventArgs e)
        {
            float t = (float)DateTime.Now.TimeOfDay.TotalSeconds;
            float pos = (float)(10 * Math.Sin(t));
            float vel = 5.0f;
            float kp = 100.0f;
            float kd = 1.0f;
            float tor = 0.0f;

            UInt16 pos_tmp = dm_can.float_to_uint(pos, P_MIN, P_MAX, 16);
            UInt16 vel_tmp = dm_can.float_to_uint(vel, V_MIN, V_MAX, 12);
            UInt16 kp_tmp = dm_can.float_to_uint(kp, KP_MIN, KP_MAX, 12);
            UInt16 kd_tmp = dm_can.float_to_uint(kd, KD_MIN, KD_MAX, 12);
            UInt16 tor_tmp = dm_can.float_to_uint(tor, T_MIN, T_MAX, 12);

            byte[] mitFrame = new byte[8];
            mitFrame[0] = (byte)(pos_tmp >> 8);
            mitFrame[1] = (byte)(pos_tmp & 0xFF);
            mitFrame[2] = (byte)((vel_tmp >> 4) & 0xFF);
            mitFrame[3] = (byte)(((vel_tmp & 0xF) << 4) | ((kp_tmp >> 8) & 0xF));
            mitFrame[4] = (byte)(kp_tmp & 0xFF);
            mitFrame[5] = (byte)(kd_tmp >> 4);
            mitFrame[6] = (byte)(((kd_tmp & 0xF) << 4) | ((tor_tmp >> 8) & 0xF));
            mitFrame[7] = (byte)(tor_tmp & 0xFF);

            dm_can.ControlSendtoCAN(serialPort, mitFrame, canId, 0, 1, 1, true);
            Console.WriteLine($"[MIT] POS = {pos:F2}");
        }

        private void Form1_Load(object sender, EventArgs e)
        {
            // You can ignore this or use it later
        }
    }
}
