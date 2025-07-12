using System;
using System.Drawing;
using System.Windows.Forms;

namespace auxApp
{
    public class FlightSimulatorPanel : Panel
    {
        public FlightSimulatorPanel()
        {
            this.DoubleBuffered = true;
            this.BackColor = Color.White;
        }

        protected override void OnPaint(PaintEventArgs e)
        {
            base.OnPaint(e);
            // TODO: 绘制姿态仪和纸飞机
            e.Graphics.DrawString("Flight Attitude Indicator & Paper Plane View", this.Font, Brushes.Gray, 10, 10);
        }
    }
} 