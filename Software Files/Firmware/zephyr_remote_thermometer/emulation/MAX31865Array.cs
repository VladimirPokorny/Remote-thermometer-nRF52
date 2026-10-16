//
// Renode model of the thermometer's four MAX31865 RTD-to-digital converters on one SPI bus.
//
// Each chip is selected by its own chip-select GPIO (active low) and powered through a load
// switch (EN, active high). The model answers SPI register reads/writes for the selected chip
// and turns the configured channel temperature into an RTD code using the Callendar-Van Dusen
// equation for a PT100/PT1000 and the reference resistor, like the real circuit would.
// As on the PCB, the load switch powers the chip: while EN is off it doesn't answer, and it starts
// from its reset configuration every time EN turns on.
//
// GPIO inputs: 0..3 = CS_1..CS_4, 4..7 = EN_1..EN_4 (see max31865.repl).
//
// Change temperatures at run time from the Renode monitor, e.g.:
//   sysbus.spi2.max31865 Temperature1 85.5
//   sysbus.spi2.max31865 SetTemperature 2 -10
//   sysbus.spi2.max31865 SetFault 3 0x84        (fault status bits for sensor 3; 0 clears)
//
using System;
using Antmicro.Renode.Core;
using Antmicro.Renode.Logging;

namespace Antmicro.Renode.Peripherals.SPI
{
    public class MAX31865Array : ISPIPeripheral, IGPIOReceiver
    {
        public MAX31865Array(double rtdNominal = 100, double refResistor = 430)
        {
            RtdNominal = rtdNominal;
            RefResistor = refResistor;
            temperatures = new double[] { 21.5, 85.0, 150.0, -20.0 };
            injectedFault = new byte[ChipCount];
            chips = new Chip[ChipCount];
            selected = new bool[ChipCount];
            powered = new bool[ChipCount];
            for(var i = 0; i < ChipCount; i++)
            {
                chips[i] = new Chip();
            }
            Reset();
        }

        public void Reset()
        {
            for(var i = 0; i < ChipCount; i++)
            {
                chips[i].Reset();
                selected[i] = false;
                powered[i] = false;
            }
        }

        public void OnGPIO(int number, bool value)
        {
            if(number >= 0 && number < ChipCount)
            {
                // CS is active low; a falling edge starts a new transaction
                var nowSelected = !value;
                if(nowSelected && !selected[number])
                {
                    chips[number].BeginTransaction();
                }
                selected[number] = nowSelected;
            }
            else if(number >= ChipCount && number < 2 * ChipCount)
            {
                // The load switch powers the whole channel: the chip starts from its reset state
                var ch = number - ChipCount;
                if(value != powered[ch])
                {
                    this.Log(LogLevel.Debug, "Sensor {0}: load switch {1}", ch + 1, value ? "on" : "off");
                    if(value)
                    {
                        chips[ch].Reset();
                    }
                }
                powered[ch] = value;
            }
            else
            {
                this.Log(LogLevel.Warning, "Unexpected GPIO input {0}", number);
            }
        }

        public byte Transmit(byte data)
        {
            var ch = SelectedChip();
            if(ch < 0)
            {
                // Nobody drives MISO
                return 0x00;
            }
            if(!powered[ch])
            {
                // Unpowered chip behind an unpowered bus buffer: nothing answers, writes are lost
                this.Log(LogLevel.Warning, "Sensor {0}: SPI access with the load switch off", ch + 1);
                return 0x00;
            }
            return chips[ch].Transmit(data, () => Convert(ch));
        }

        public void FinishTransmission()
        {
            // Transactions are framed by the chip-select GPIOs
        }

        public void SetTemperature(int sensor, double celsius)
        {
            temperatures[CheckSensor(sensor)] = celsius;
        }

        public double GetTemperature(int sensor)
        {
            return temperatures[CheckSensor(sensor)];
        }

        public void SetFault(int sensor, byte faultStatus)
        {
            injectedFault[CheckSensor(sensor)] = faultStatus;
        }

        public double Temperature1 { get { return temperatures[0]; } set { temperatures[0] = value; } }
        public double Temperature2 { get { return temperatures[1]; } set { temperatures[1] = value; } }
        public double Temperature3 { get { return temperatures[2]; } set { temperatures[2] = value; } }
        public double Temperature4 { get { return temperatures[3]; } set { temperatures[3] = value; } }

        // Resistance of the RTD at 0 C (100 for PT100, 1000 for PT1000)
        public double RtdNominal { get; set; }
        // Reference resistor between REFIN+ and REFIN-
        public double RefResistor { get; set; }

        private int SelectedChip()
        {
            var found = -1;
            for(var i = 0; i < ChipCount; i++)
            {
                if(!selected[i])
                {
                    continue;
                }
                if(found >= 0)
                {
                    this.Log(LogLevel.Warning, "Sensors {0} and {1} selected at once, bus contention", found + 1, i + 1);
                    return -1;
                }
                found = i;
            }
            return found;
        }

        private int CheckSensor(int sensor)
        {
            if(sensor < 1 || sensor > ChipCount)
            {
                throw new ArgumentException(string.Format("Sensor must be 1..{0}", ChipCount));
            }
            return sensor - 1;
        }

        // One-shot conversion: 15-bit ADC code = R_rtd / R_ref * 2^15, fault flag in bit 0 of the LSB
        private void Convert(int ch)
        {
            var chip = chips[ch];
            byte fault = injectedFault[ch];

            if((chip.Config & ConfigBias) == 0)
            {
                this.Log(LogLevel.Warning, "Sensor {0}: conversion with the bias voltage off", ch + 1);
                fault |= RtdInLow;
            }

            ushort code = 0;
            if((fault & RtdInLow) == 0)
            {
                var ratio = RtdResistance(temperatures[ch]) / RefResistor;
                code = (ushort)Math.Max(0, Math.Min(0x7FFF, Math.Round(ratio * 32768)));
            }

            var rtd = (ushort)((code << 1) | (fault != 0 ? 1 : 0));
            chip.Registers[RtdMsb] = (byte)(rtd >> 8);
            chip.Registers[RtdLsb] = (byte)rtd;
            chip.Registers[FaultStatus] |= fault;

            this.Log(LogLevel.Debug, "Sensor {0}: {1:0.###} C -> code {2} (0x{3:X4}), fault 0x{4:X2}",
                ch + 1, temperatures[ch], code, rtd, fault);
        }

        // Callendar-Van Dusen (IEC 60751)
        private double RtdResistance(double t)
        {
            const double A = 3.9083e-3;
            const double B = -5.775e-7;
            const double C = -4.183e-12;

            var r = 1 + A * t + B * t * t;
            if(t < 0)
            {
                r += C * (t - 100) * t * t * t;
            }
            return RtdNominal * r;
        }

        private readonly double[] temperatures;
        private readonly byte[] injectedFault;
        private readonly Chip[] chips;
        private readonly bool[] selected;
        private readonly bool[] powered;

        private const int ChipCount = 4;

        private const int Config = 0x00;
        private const int RtdMsb = 0x01;
        private const int RtdLsb = 0x02;
        private const int FaultStatus = 0x07;

        private const byte ConfigBias = 0x80;
        private const byte ConfigOneShot = 0x20;
        private const byte ConfigFaultClear = 0x02;
        private const byte RtdInLow = 0x08;

        private class Chip
        {
            public byte[] Registers = new byte[8];

            public byte Config { get { return Registers[0]; } }

            public void Reset()
            {
                Array.Clear(Registers, 0, Registers.Length);
                // High fault threshold resets to 0xFFFF, low to 0x0000
                Registers[3] = 0xFF;
                Registers[4] = 0xFF;
                expectAddress = true;
            }

            public void BeginTransaction()
            {
                expectAddress = true;
            }

            // First byte after CS falls is the address (bit 7 = write), then data with auto-increment
            public byte Transmit(byte data, Action convert)
            {
                if(expectAddress)
                {
                    expectAddress = false;
                    write = (data & 0x80) != 0;
                    address = data & 0x07;
                    return 0x00;
                }

                byte result = 0x00;
                if(write)
                {
                    WriteRegister(address, data, convert);
                }
                else
                {
                    result = Registers[address];
                }
                address = (address + 1) & 0x07;
                return result;
            }

            private void WriteRegister(int reg, byte value, Action convert)
            {
                switch(reg)
                {
                    case 0x00:
                        Registers[0] = (byte)(value & ~(ConfigOneShot | ConfigFaultClear));
                        if((value & ConfigFaultClear) != 0)
                        {
                            Registers[FaultStatus] = 0;
                        }
                        if((value & ConfigOneShot) != 0)
                        {
                            convert();
                        }
                        break;
                    case 0x03:
                    case 0x04:
                    case 0x05:
                    case 0x06:
                        Registers[reg] = value;
                        break;
                    default:
                        // RTD and fault status registers are read-only
                        break;
                }
            }

            private bool expectAddress = true;
            private bool write;
            private int address;
        }
    }
}
