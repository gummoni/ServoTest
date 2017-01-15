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
                //servo.ControlTorque(-0.2);
                servo.ControlPosition(100);
                servo.OnNext();
            }

            Console.WriteLine("--end--");
            Console.ReadKey();
        }
    }

    /// <summary>
    /// １回転4096（内部１Tickあたり１００分解能(11bit:2048 * 4現象 * 50(ギヤ比))）
    /// エンコーダレベルで100Tick＝軸座標１Tick
    /// 電気レベルで4096Tick = エンコーダ256Tick
    /// 
    /// 
    /// 電気4:エンコーダ1
    /// エンコーダ50:実角度1
    /// 電気200:エンコーダ50:実角度1
    /// 
    /// 電気200*4096:エンコーダ50*4096:実角度1*4096
    /// エンコーダ1Bit削って
    /// 電気100*4096:エンコーダ50*2048:実角度1*4096
    /// 
    /// </summary>
    public class Servo
    {
        //サーボステータス
        public double Position { get; private set; }    //現在位置
        public double Speed { get; private set; }       //現在速度
        public double Power { get; set; } = 0.0;        //現在モーター出力

        //内部パラメータ
        double Diff { get; set; }                       //加速度補正
        double ControlAcc { get; set; }                 //目標加速度
        double ResultAcc { get; set; }                  //実測加速度

        //設定値
        const double LimitTorque = 50.0;                //トルク制限値
        const double TargetSpeed = 3.0;                 //目標速度
        const double TargetAcc = 1.0;                   //目標加速度(P)
        const double TargetDec = 1.3;                   //目標減速度(P)
        const double IGain = 2.0;                       //Iゲイン
        const double DGain　= 3.0;                      //Dゲイン

        //固有特性
        const double MotorGain = 1.2;                   //最大トルク出力値

        /// <summary>
        /// 位置制御
        /// </summary>
        /// <param name="goalPosition"></param>
        public void ControlPosition(double goalPosition)
        {
            var restPosition = goalPosition - Position;     //残り移動量
            ControlSpeed(restPosition);
        }

        /// <summary>
        /// 速度制御(PTP動作を実現)
        /// X軸：時間軸、Y軸：目標速度な台形[／￣￣＼]となるような速度を調整する
        /// 加速区間：目標速度に到達するように目標加速値で加速する
        /// 減速区間：目標速度が現在速度 - 目標減速値となるように減速する
        /// </summary>
        /// <param name="targetSpeed"></param>
        public void ControlSpeed(double restPosition)
        {
            // 残りX値の求め方
            // ピタゴラスの定理より
            // (X+1)*Y/2 ＝ Rest
            // (X+1)*Y   ＝ 2 * Rest
            // Y=AX, A=DECより
            // (X+1)*X   ＝ 2 * Rest / DEC
            //  X^2      ≒ 2 * Rest / DEC
            //         X ≒ √(2 * Rest / DEC)

            //PTP動作(時間軸Xを基準として加減速を行う)
            var rest = Math.Abs(restPosition);                      //残り移動量(絶対値)
            var hugo2 = (0 <= restPosition) ? +1.0 : -1.0;          //移動したい方向
            var MX = TargetSpeed / TargetDec;                       //減速開始位置X
            var ZX = Math.Sqrt(2 * rest / TargetDec);               //現在位置X

            if ((MX + 1) < ZX)
            {
                //加速区間(速度制限付き、加速カーブは加速上限で表現)
                var spd = hugo2 * Math.Min(TargetAcc, TargetSpeed - Math.Abs(Speed));
                ControlTorque(spd);
            }
            else if (TargetDec < rest)
            {
                //減速区間(減速カーブは時系列で表現するので、目標速度にめがけてブレーキする)
                var targetspeed = hugo2 * (ZX - 1.0) * TargetDec;
                var spd = targetspeed - Speed;
                ControlTorque(spd);
            }
            else
            {
                //目標値まで微調整
                var targetspeed = restPosition / DGain;
                var spd = targetspeed - Speed;
                ControlTorque(spd);
            }
        }

        /// <summary>
        /// 出力トルク制御(速度制御より指示された加速度を正確に実現する)
        /// </summary>
        /// <param name="acc">加速度</param>
        public void ControlTorque(double targetAcc)
        {
            ControlAcc = targetAcc;             //目標加速度（この値は補正値Diffの計算用に使用する）
            Power += targetAcc + Diff;
            //トルク出力制限
            if (+LimitTorque < Power) Power = LimitTorque;
            if (-LimitTorque > Power) Power = -LimitTorque;
        }

        void ControlServo(int current, double power)
        {
            //TODO:サーボ制御処理はダミーコード

            // 分解能は12bit（４０９６）だけどノイズ対策で1Bit減らして11Bit
            // ４現像なので１回転 11 Bit * 2bit = 11bit (8192) * 4 = 32768 = １回転
            // ギア比50:1なので
            // 4096(12bit) * 100
            // １回転0~4095 モーター内部分解能は100分解能

            //サーボON中は必ずどちらかに出力かつ現在位置から±９０度にUVW出力
            bool cw = (0 < power) || ((0 == power) && (0 < Diff));
            if (cw)
            {
                //CW
                var U = power * Math.Cos((current + 1024) % 4096);  // Cur + 90
                var V = power * Math.Cos((current + 2389) % 4096);  // Cur + 210
                var W = power * Math.Cos((current + 3755) % 4096);  // Cur + 330
            }
            else
            {
                //CCW
                var U = power * Math.Cos((current + 3072) % 4096);  // Cur + 270
                var V = power * Math.Cos((current + 341) % 4096);   // Cur + 30
                var W = power * Math.Cos((current + 1707) % 4096);  // Cur + 150
            }
        }


        static Random rnd = new Random();
        /// <summary>
        /// 実行
        /// /// </summary>
        public void OnNext()
        {
            var field = GetFieldPower();
            var nextPosition = Position + Power * MotorGain + field;    // 外乱の影響の付加
            FeedBack(nextPosition);
            Console.WriteLine($"Pos={Position}, Spd={Speed}, Pwr={Power}, Acc={ResultAcc}");
        }

        /// <summary>
        /// 環境負荷
        /// </summary>
        /// <returns></returns>
        double GetFieldPower()
        {
            var random1 = (50 - Position) / 10;
            var random2 = -Speed / 10;
            var random3 = (rnd.Next(0, 200) - 100) / 10000.0;   //エンコーダ誤差はモーター１Tickの１００分の１まで
            //random1 = random2 = 0.0;
            random3 = 0;
            return random1 + random2 + random3;
        }

        /// <summary>
        /// フィードバック
        /// </summary>
        /// <param name="nextPosition"></param>
        void FeedBack(double nextPosition)
        {
            var speed = nextPosition - Position;
            var acc = speed - Speed;
            var diff = ControlAcc - acc;
            Position = nextPosition;
            Speed = speed;
            ResultAcc = acc;
            Diff += diff / IGain;
        }
    }
}
