using System;
using System.IO.Ports;
using System.Windows.Forms;



namespace Test1
{
    public partial class Form1 : Form
    {
        SerialPort serialport;
        public Form1()
        {
            InitializeComponent();

            serialport = new SerialPort("COM4", 921600);
        }

        private void button1_Click(object sender, EventArgs e)
        {
            try
            {
                if (!serialport.IsOpen)
                {
                    serialport.Open();
                    MessageBox.Show("✅ Connected to COM4 at 921600 baud");
                }
                else
                {
                    MessageBox.Show("⚠️ Already connected");
                }
            }
            catch (Exception ex)
            {
                MessageBox.Show("❌ Connection failed: " + ex.Message);
            }
        }
    }
}
