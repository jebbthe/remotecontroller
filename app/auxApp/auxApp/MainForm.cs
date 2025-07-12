using System;
using System.Drawing;
using System.Windows.Forms;
using System.IO.Ports;

namespace auxApp
{
    public partial class MainForm : Form
    {
        private SerialPort serialPort;
        private FlightData currentFlightData;

        public MainForm()
        {
            InitializeComponent();
            InitSerialPortConfig();
        }

        private void InitSerialPortConfig()
        {
            comboBoxPort.Items.AddRange(SerialPort.GetPortNames());
            if (comboBoxPort.Items.Count > 0) comboBoxPort.SelectedIndex = 0;
            comboBoxBaud.Items.AddRange(new object[] { "115200", "57600", "38400", "9600" });
            comboBoxBaud.SelectedIndex = 0;
            buttonOpenClose.Click += ButtonOpenClose_Click;
        }

        private void ButtonOpenClose_Click(object sender, EventArgs e)
        {
            if (serialPort == null || !serialPort.IsOpen)
            {
                try
                {
                    serialPort = new SerialPort(comboBoxPort.Text, int.Parse(comboBoxBaud.Text));
                    serialPort.DataReceived += SerialPort_DataReceived;
                    serialPort.Open();
                    labelSerialStatus.Text = "已连接";
                    buttonOpenClose.Text = "关闭串口";
                }
                catch (Exception ex)
                {
                    MessageBox.Show("串口打开失败: " + ex.Message);
                }
            }
            else
            {
                try
                {
                    serialPort.Close();
                    labelSerialStatus.Text = "未连接";
                    buttonOpenClose.Text = "打开串口";
                }
                catch (Exception ex)
                {
                    MessageBox.Show("串口关闭失败: " + ex.Message);
                }
            }
        }

        private void SerialPort_DataReceived(object sender, SerialDataReceivedEventArgs e)
        {
            try
            {
                string line = serialPort.ReadLine();
                this.BeginInvoke(new Action(() =>
                {
                    textBoxRawData.Text = line.Trim();
                    // TODO: 解析数据并存入currentFlightData
                }));
            }
            catch { }
        }

        private class FlightData
        {
            public float Roll, Pitch, Yaw;
            public float Left, Right, Throttle;
            // 可根据实际协议扩展
        }
    }
} 