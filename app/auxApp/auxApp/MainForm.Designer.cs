namespace auxApp
{
    partial class MainForm
    {
        /// <summary>
        /// 必需的设计器变量。
        /// </summary>
        private System.ComponentModel.IContainer components = null;

        /// <summary>
        /// 清理所有正在使用的资源。
        /// </summary>
        /// <param name="disposing">如果应释放托管资源，为 true；否则为 false。</param>
        protected override void Dispose(bool disposing)
        {
            if (disposing && (components != null))
            {
                components.Dispose();
            }
            base.Dispose(disposing);
        }

        private FlightSimulatorPanel simulatorPanel;
        private System.Windows.Forms.ComboBox comboBoxPort;
        private System.Windows.Forms.ComboBox comboBoxBaud;
        private System.Windows.Forms.Button buttonOpenClose;
        private System.Windows.Forms.Label labelSerialStatus;
        private System.Windows.Forms.TextBox textBoxRawData;

        #region Windows 窗体设计器生成的代码

        /// <summary>
        /// 设计器支持所需的方法 - 不要
        /// 使用代码编辑器修改此方法的内容。
        /// </summary>
        private void InitializeComponent()
        {
            this.simulatorPanel = new FlightSimulatorPanel();
            this.comboBoxPort = new System.Windows.Forms.ComboBox();
            this.comboBoxBaud = new System.Windows.Forms.ComboBox();
            this.buttonOpenClose = new System.Windows.Forms.Button();
            this.labelSerialStatus = new System.Windows.Forms.Label();
            this.textBoxRawData = new System.Windows.Forms.TextBox();
            this.SuspendLayout();
            // 
            // simulatorPanel
            // 
            this.simulatorPanel.Dock = System.Windows.Forms.DockStyle.Fill;
            this.simulatorPanel.Location = new System.Drawing.Point(0, 0);
            this.simulatorPanel.Name = "simulatorPanel";
            this.simulatorPanel.Size = new System.Drawing.Size(1200, 800);
            this.simulatorPanel.TabIndex = 0;
            // 
            // MainForm
            // 
            this.AutoScaleDimensions = new System.Drawing.SizeF(8F, 20F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.ClientSize = new System.Drawing.Size(1200, 800);
            this.Controls.Add(this.simulatorPanel);
            // 串口号
            this.comboBoxPort.DropDownStyle = System.Windows.Forms.ComboBoxStyle.DropDownList;
            this.comboBoxPort.Location = new System.Drawing.Point(12, 12);
            this.comboBoxPort.Size = new System.Drawing.Size(80, 20);
            // 波特率
            this.comboBoxBaud.DropDownStyle = System.Windows.Forms.ComboBoxStyle.DropDownList;
            this.comboBoxBaud.Location = new System.Drawing.Point(100, 12);
            this.comboBoxBaud.Size = new System.Drawing.Size(80, 20);
            // 打开/关闭按钮
            this.buttonOpenClose.Location = new System.Drawing.Point(190, 10);
            this.buttonOpenClose.Size = new System.Drawing.Size(60, 24);
            this.buttonOpenClose.Text = "打开串口";
            // 状态
            this.labelSerialStatus.Location = new System.Drawing.Point(260, 14);
            this.labelSerialStatus.Size = new System.Drawing.Size(100, 16);
            this.labelSerialStatus.Text = "未连接";
            // 原始数据
            this.textBoxRawData.Location = new System.Drawing.Point(12, 40);
            this.textBoxRawData.Size = new System.Drawing.Size(350, 20);
            this.textBoxRawData.ReadOnly = true;
            // 添加控件
            this.Controls.Add(this.comboBoxPort);
            this.Controls.Add(this.comboBoxBaud);
            this.Controls.Add(this.buttonOpenClose);
            this.Controls.Add(this.labelSerialStatus);
            this.Controls.Add(this.textBoxRawData);
            this.Name = "MainForm";
            this.Text = "Flight Attitude Simulator";
            this.ResumeLayout(false);
        }

        #endregion
    }
} 