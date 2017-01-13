using System;

namespace ConsoleApplication1
{
    class Program
    {
        static void Main(string[] args)
        {
            var servo = new Servo();
            for (var i = 0; i < 1300; i++)
            {
                servo.ControlPosition(370);
                servo.OnNext();
            }

            Console.ReadKey();
        }
    }

    public enum ControlMode
    {
        Acc,
        Dec,
        Stop,
    }

    public class Servo
    {
        public Motor Old { get; set; } = new Motor();   //前回位置
        public Motor Cur { get; set; } = new Motor();   //現在位置
        public double Power { get; set; } = 0.0;        //現在モーター出力
        ControlMode OldControl = ControlMode.Stop;      //前回の制御パターン

        const double MaxTorque = 0.5;                   //最大トルク出力値
        const double LimitTorque = 10.0;                //トルク制限値
        const double TargetSpeed = 10.0;                //目標速度
        const double TargetAcc = 1.0;                   //目標加速度
        const double TargetDec = 2.3;                   //目標減速度

        /// <summary>
        /// 位置制御(所定の位置へ移動）
        /// </summary>
        /// <param name="targetPosition"></param>
        public void ControlPosition(double targetPosition)
        {
            ControlSpeed(targetPosition - Cur.Position);
        }

        /// <summary>
        /// 速度制御(PTP動作を実現)
        /// </summary>
        /// <param name="targetSpeed"></param>
        public void ControlSpeed(double restPosition)
        {
            //PTP動作
            //加減速
            var LX = 1 + Math.Abs((Cur.Speed + Cur.ResultAcc) / TargetDec);         //現在速度
            var limitPoint = Math.Abs(LX * Cur.Speed) / 2 + Math.Abs(Cur.Speed);    //減速開始ポイント
            var hugo1 = (0 <= Cur.Speed) ? +1.0 : -1.0;                             //現在の進行方向
            var hugo2 = (0 <= restPosition) ? +1.0 : -1.0;                          //移動したい方向
            var delt = Math.Abs(restPosition) - limitPoint;                         //減速開始ポイントまでの距離

            if (0 == delt)
            {
                //目標位置到達
                ControlTorque(0.0);
                OldControl = ControlMode.Stop;

                if (0 == Cur.Speed)
                {
                    //デバッグ用チェックポイント
                    Console.WriteLine("Finish");
                    Console.ReadKey();
                }
            }
            else if (0 < delt)
            {
                //減速開始ポイントまで加速
                if (OldControl == ControlMode.Dec)
                {
                    //現状維持(減速からすぐ加速しないよう１回休み)
                    ControlTorque(0.0);
                    OldControl = ControlMode.Stop;
                }
                else
                {
                    ControlTorque(+hugo2 * Math.Min(TargetAcc, Math.Abs(delt)));
                    OldControl = ControlMode.Acc;
                }
            }
            else
            {
                //減速開始ポイント内なので減速
                ControlTorque(-hugo1 * Math.Min(TargetDec, Math.Abs(Cur.Speed)));
                OldControl = ControlMode.Dec;
            }
        }

        /// <summary>
        /// 出力トルク制御(速度制御より指示された加速度を正確に実現する)
        /// </summary>
        /// <param name="acc">加速度</param>
        public void ControlTorque(double targetAcc)
        {
            Cur.ControlAcc = targetAcc;
            Power += Cur.Torque * targetAcc;
            //トルク出力制限
            if (+LimitTorque < Power) Power = LimitTorque;
            if (-LimitTorque > Power) Power = -LimitTorque;
        }

        /// <summary>
        /// 実行
        /// </summary>
        public void OnNext()
        {
            var random1 = (50 - Cur.Position) / 1000;
            var random2 = -Cur.Speed / 800;
            //random1 = random2 = 0.0;
            var nextPosition = Cur.Position + Power * MaxTorque + random1 + random2;      // 外乱の影響の付加
            FeedBack(nextPosition);
            Console.WriteLine($"Pos={Cur.Position}, Spd={Cur.Speed}, Pwr={Power}, Acc={Cur.ResultAcc}");
            //Console.WriteLine(Cur.Speed);
        }

        /// <summary>
        /// フィードバック（センサ入力）
        /// </summary>
        /// <param name="nextPosition"></param>
        void FeedBack(double nextPosition)
        {
            var nxt = new Motor();
            nxt.Position = nextPosition;
            nxt.Speed = nextPosition - Cur.Position;
            nxt.ResultAcc = nxt.Speed - Cur.Speed;
            nxt.Torque = (0.0 == Cur.ControlAcc) ? 1.0 : (1.0 - (nxt.ResultAcc - Cur.ControlAcc) / Cur.ControlAcc);
            Old = Cur;
            Cur = nxt;
        }
    }

    public class Motor
    {
        //フィードバック時に入力
        public double Position { get; set; }                                // 現在位置
        public double Speed { get; set; }                                   // 現在速度
        public double ResultAcc { get; set; }                               // フィードバック加速度
        public double Torque { get; set; } = 1.0;                           // 現在トルク

        //設定する
        public double ControlAcc { get; set; }                              // コントロール加速度
    }

}
