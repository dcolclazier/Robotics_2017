using System;
using System.Threading;
using Microsoft.SPOT;
using Microsoft.SPOT.Hardware;
using SecretLabs.NETMF.Hardware.Netduino;
using Math = System.Math;

namespace Robotics2017 {

   
    public enum Connector
    {
        A,
        B
    }
    public interface IStepSequencer
    {
        void PerformStep(int direction);

        void ReleaseHoldingTorque();
    }
    public class TwoPhaseMicrosteppingSequencer : IStepSequencer
    {
        private readonly int _maxIndex;
        private readonly HBridge _phase1;
        private readonly HBridge _phase2;
        private double[] _inPhaseDutyCycle;
        private double[] _outOfPhaseDutyCycle;
        private int _phaseIndex;

        public TwoPhaseMicrosteppingSequencer(HBridge phase1Bridge, HBridge phase2Bridge, int stepsPerStepCycle)
        {
            _phase1 = phase1Bridge;
            _phase2 = phase2Bridge;
            _maxIndex = stepsPerStepCycle - 1;
            _phaseIndex = 0;
            ConfigureStepTables(stepsPerStepCycle);
        }

        private void ConfigureStepTables(int microsteps)
        {
            if (microsteps == 4)
                ComputeWholeStepTables();
            else if (microsteps == 8)
            {
                ComputeHalfStepTables();
            }
            else
            {
                if (microsteps < 8)
                    throw new ArgumentException("Use 4 for full steps; 8 for half steps; or >8 for stepsPerStepCycle", "microsteps");
                ComputeMicrostepTables(microsteps);
            }
        }

        private void ComputeHalfStepTables()
        {
            _inPhaseDutyCycle = new double[8]
            {
        1.0,
        1.0,
        0.0,
        -1.0,
        -1.0,
        -1.0,
        0.0,
        1.0
            };
            _outOfPhaseDutyCycle = new double[8]
            {
        0.0,
        1.0,
        1.0,
        1.0,
        0.0,
        -1.0,
        -1.0,
        -1.0
            };
        }

        private void ComputeWholeStepTables()
        {
            _inPhaseDutyCycle = new double[4]
            {
        1.0,
        -1.0,
        -1.0,
        1.0
            };
            _outOfPhaseDutyCycle = new double[4]
            {
        1.0,
        1.0,
        -1.0,
        -1.0
            };
        }

        public void PerformStep(int direction)
        {
            _phaseIndex += direction;
            if (_phaseIndex > _maxIndex)
                _phaseIndex = 0;
            if (_phaseIndex < 0)
                _phaseIndex = _maxIndex;
            _phase1.SetOutputPowerAndPolarity(_inPhaseDutyCycle[_phaseIndex]);
            _phase2.SetOutputPowerAndPolarity(_outOfPhaseDutyCycle[_phaseIndex]);
        }

        public void ReleaseHoldingTorque()
        {
            _phase1.SetOutputPowerAndPolarity(0.0);
            _phase2.SetOutputPowerAndPolarity(0.0);
        }

        private void ComputeMicrostepTables(int microsteps)
        {
            double num1 = 2.0 * Math.PI / (microsteps - 1);
            _inPhaseDutyCycle = new double[microsteps];
            _outOfPhaseDutyCycle = new double[microsteps];
            for (int index = 0; index < microsteps; ++index)
            {
                double num2 = index * num1;
                _inPhaseDutyCycle[index] = Math.Sin(num2);
                _outOfPhaseDutyCycle[index] = Math.Cos(num2);
            }
        }
    }
    public class SimpleHBridge : HBridge
    {
        private readonly OutputPort _direction;
        private readonly PWM _power;

        public SimpleHBridge(Cpu.PWMChannel powerControlChannel, Cpu.Pin directionControlPin)
        {
            _direction = new OutputPort(directionControlPin, false);
            _power = new PWM(powerControlChannel, 500.0, 0.0, false);
            _power.Start();
        }

        public override void SetOutputPowerAndPolarity(double duty)
        {
            bool polarity = duty >= 0.0;
            SetOutputPowerAndPolarity(Math.Abs(duty), polarity);
            base.SetOutputPowerAndPolarity(duty);
        }

        private void SetOutputPowerAndPolarity(double magnitude, bool polarity)
        {
            if (polarity != Polarity)
                _power.DutyCycle = 0.0;
            _direction.Write(polarity);
            _power.DutyCycle = magnitude;
        }
    }
    public abstract class HBridge
        {
            private double _duty;

            public bool Polarity => _duty >= 0.0;

            public double Power => Math.Abs(_duty);

            public virtual void SetOutputPowerAndPolarity(double duty)
            {
                if (duty > 1.0 || duty < -1.0)
                    throw new ArgumentOutOfRangeException("duty", "-1.0 to 1.0 inclusive");
                _duty = duty;
            }

            public virtual void ApplyBrake()
            {
                Debug.Print("Apply Brake");
                SetOutputPowerAndPolarity(0.0);
            }

            public virtual void ReleaseTorque()
            {
                Debug.Print("Release torque");
                SetOutputPowerAndPolarity(0.0);
            }
        }
    public class DcMotor
    {
        private const int DefaultResolution = 100;
        private Timer _accelerationTimer;
        private long _startTime;
        private double _startVelocity;
        private double _targetVelocity;
        private readonly int _accelerationResolutionInMilliseconds;
        private readonly HBridge _motorWinding;

        public double CurrentVelocity { get; private set; }

        public double Acceleration { get; private set; }

        public DcMotor(HBridge motorWinding, int accelerationResolutionInMilliseconds = 100)
        {
            Acceleration = 0.2;
            _motorWinding = motorWinding;
            _accelerationResolutionInMilliseconds = accelerationResolutionInMilliseconds;
        }

        public void AccelerateToVelocity(double velocity)
        {
            Debug.Print("Accelerate to: " + velocity.ToString("F4"));
            StopAccelerating();
            _startTime = DateTime.UtcNow.Ticks;
            _startVelocity = CurrentVelocity;
            _targetVelocity = velocity;
            StartAccelerating();
        }

        private void StartAccelerating()
        {
            Debug.Print("Start acceleration timer");
            if (_accelerationTimer == null)
                _accelerationTimer = new Timer(HandleAccelerationTimerTick, null, _accelerationResolutionInMilliseconds, _accelerationResolutionInMilliseconds);
            else
                _accelerationTimer.Change(_accelerationResolutionInMilliseconds, _accelerationResolutionInMilliseconds);
        }

        private void StopAccelerating()
        {
            Debug.Print("Stop acceleration timer");
            if (_accelerationTimer == null)
                return;
            _accelerationTimer.Change(-1, -1);
        }

        private void HandleAccelerationTimerTick(object ignored)
        {
            int num = Math.Sign(_targetVelocity - CurrentVelocity);
            double acceleratedVelocity = ComputeAcceleratedVelocity(Acceleration * num);
            double duty = num >= 0 ? Accelerate(acceleratedVelocity) : Decelerate(acceleratedVelocity);
            Debug.Print("Velocity " + duty.ToString("F4"));
            _motorWinding.SetOutputPowerAndPolarity(duty);
            CurrentVelocity = duty;
        }

        private double Decelerate(double newVelocity)
        {
            if (newVelocity > _targetVelocity)
                return newVelocity;
            StopAccelerating();
            return _targetVelocity;
        }

        private double Accelerate(double newVelocity)
        {
            if (newVelocity < _targetVelocity)
                return newVelocity;
            StopAccelerating();
            return _targetVelocity;
        }

        private double ComputeAcceleratedVelocity(double acceleration)
        {
            double num = (DateTime.UtcNow.Ticks - _startTime) / 10000000.0;
            return _startVelocity + acceleration * num;
        }
    }
    public sealed class SparkfunArdumoto
    {
        public HBridge GetHBridge(Connector winding)
        {
            return Netduino2BridgeConfiguration(winding);
            
        }

        

        private static HBridge Netduino2BridgeConfiguration(Connector winding)
        {
            switch (winding)
            {
                case Connector.A:
                    return new SimpleHBridge(PWMChannels.PWM_PIN_D3, Pins.GPIO_PIN_D12);
                case Connector.B:
                    return new SimpleHBridge(PWMChannels.PWM_PIN_D11, Pins.GPIO_PIN_D13);
                default:
                    throw new ArgumentOutOfRangeException("winding");
            }
        }

        public IStepSequencer GetMicrosteppingStepperMotor(int microsteps, HBridge phase1, HBridge phase2)
        {
            return new TwoPhaseMicrosteppingSequencer(phase1, phase2, microsteps);
        }

        public void InitializeShield()
        {
        }

        public HBridge GetDcMotor(Connector connector)
        {
            return GetHBridge(connector);
        }
    }

    public class IrDistanceSensor {
        private AnalogInput _input;
        private double[] _data;

        public IrDistanceSensor(Cpu.AnalogChannel channel, int sampleCount) {
            _input = new AnalogInput(channel);
            _data = new double[sampleCount];
        }

        public double ReadDistance() {

            var total = 0d;
            var min = double.MaxValue;
            var max = double.MinValue;
            var count = _data.Length;

            for (int i = 0; i < count; i++) {
                var raw = _input.Read();
                _data[i] = raw;
                total += raw;

                if (raw < min)min = raw;
                if (raw > max) max = raw;

            }

            var average = total/count;
            var stDev = StandardDeviation(_data, average);

            if (stDev > 0.01) {
                var position = 0;
                min = average - stDev;
                max = average + stDev;


                for (int i = 0; i < count; i++) {
                    var data = _data[i];
                    if (data > min && data < max)
                        _data[position++] = data;
                }
                average = Average(_data, position);
            }
            var volts = average*3.3;

            return 8.30330497051182 * Math.Pow(volts, 6d)
                    - 89.8292932369688 * Math.Pow(volts, 5d)
                    + 391.808954977875 * Math.Pow(volts, 4d)
                    - 885.170571942885 * Math.Pow(volts, 3d)
                    + 1106.23720332132 * Math.Pow(volts, 2d)
                    - 753.770168858126 * volts
                    + 250.173288592975;
        }
        public void Dispose()
        {
            GC.SuppressFinalize(this);
            _input.Dispose();
        }

        private static double StandardDeviation(double[] data, double avg)
        {
            var varTot = 0d;
            var max = data.Length;

            if (max == 0)
                return 0;

            for (var i = 0; i < max; i++)
            {
                var variance = data[i] - avg;
                varTot = varTot + (variance * variance);
            }

            return Math.Sqrt(varTot / max);
        }

        private static double Average(double[] data, int count)
        {
            var avg = 0d;

            for (var i = 0; i < count; i++)
                avg += data[i];

            if (avg == 0 || count == 0)
                return 0;

            return avg / count;
        }

        ~IrDistanceSensor() {
            Dispose();
        }

    }
    

    public class Program {
        public static void Main() {

            var distance = new IrDistanceSensor(AnalogChannels.ANALOG_PIN_A0,10);

            var shield = new SparkfunArdumoto();
            shield.InitializeShield();
            var bridge1 = shield.GetHBridge(Connector.A);
            var bridge2 = shield.GetHBridge(Connector.B);
            var motorA = new DcMotor(bridge1);
            var motorB = new DcMotor(bridge2);
            var random = new Random();

            while (true) {
                var speed = random.NextDouble() * 2 - 1;
                var speed2 = random.NextDouble() * 2 - 1;
                motorA.AccelerateToVelocity(speed);
                motorB.AccelerateToVelocity(speed);
                Thread.Sleep(5000);
                Debug.Print("Distance from IR Sensor: " + distance.ReadDistance());

            }
        }
    }
}


namespace System.Diagnostics
{
    //  DebuggerBrowsableState states are defined as follows:
    //      Never       never show this element
    //      Expanded    expansion of the class is done, so that all visible internal members are shown
    //      Collapsed   expansion of the class is not performed. Internal visible members are hidden
    //      RootHidden  The target element itself should not be shown, but should instead be
    //                  automatically expanded to have its members displayed.
    //  Default value is collapsed

    //  Please also change the code which validates DebuggerBrowsableState variable (in this file)
    //  if you change this enum.
    public enum DebuggerBrowsableState
    {
        Never = 0,
        Collapsed = 2,
        RootHidden = 3
    }
}
