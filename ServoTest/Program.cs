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

    //10kHzで制御
    public class BrushlessMotor
    {
        //http://toshiba.semicon-storage.com/jp/design-support/e-learning/mcupark/village/vector-1.html

        float targetD;
        float targetQ;
        PI DControl = new PI();
        PI QControl = new PI();

        public void Direction(float power, double degree)
        {
            targetD = (float)Math.Sin(degree) * power;
            targetQ = (float)Math.Cos(degree) * power;
        }

        //TODO:rm マグネットセンサの角度rm（４現象に注意）
        public void Feedback(float u, float v, float w, float rm, float re, out float U, out float V, out float W)
        {
            //クラーク変換(３つのベクトルを１つのベクトルにまとめる)
            var a = u;
            var b = (u + 2.0f * v) / Math.Sqrt(3);

            //パーク変換(時計方向に回転させ、DQ成分を求める
            var sinrm = Math.Sin(rm);
            var cosrm = Math.Cos(rm);
            var d = (float)(+a * cosrm + b * sinrm);
            var q = (float)(-a * sinrm + b * cosrm);

            //PI制御
            var D = DControl.Feedback(targetD, d);
            var Q = QControl.Feedback(targetQ, q);

            //逆パーク変換(半時計方向に回転させ、進角を決める)　進角は+90/0/-90など
            var sinre = Math.Sin(re);
            var cosre = Math.Cos(re);
            var A = D * cosre - Q * sinre;
            var B = D * sinre + Q * cosre;

            //逆クラーク変換
            U = (float)A;
            V = (float)((Math.Sqrt(3) * B - A) / 2.0f);
            W = -(u + v);

            //TODO UVW値：-1~+1の範囲に収めるようにする？
        }
    }

    public class PI
    {
        float diff;
        public float Feedback(float P, float value)
        {
            var err = P - value;
            diff += err;
            return P + diff;
        }
    }

    /// <summary>
    /// ギヤ比 1:50
    /// 分解能 14bit(エンコーダ)
    /// 現象   1:4
    /// 
    /// 
    /// 設定分解能:12bit
    /// 1pls = 200enc(エンコーダ)
    /// 1enc = 1deg(電気角12bit)
    /// 
    /// まとめ
    /// 　制御分解能(12bit, Dynamixelと同様) 単位：pls(0~4095)
    /// 　エンコーダ分解能:14bit 単位：enc(0~16385)
    /// 　三角関数テーブル:12bit 単位：deg(0~4095)=電気角
    /// 　ギヤ比1:50なので、　1pls = 200enc
    /// 　エンコーダと電気角の単位は等しい 1enc=1deg
    /// 　
    /// 　最小速度は、0.5ms周期で1deg
    /// 　  1ms  = 2deg
    /// 　  1s   = 2000deg = 2000enc = 10pls
    /// 　  1min = 600pls
    /// 　  600/4096rpm = 0.146484375rpm
    /// 　
    /// 　最大速度3000rpmの場合、
    /// 　  3000*4096/4096rpm
    /// 　  5*4096倍であることから
    /// 　  0.5ms周期で 4096*5deg
    /// </summary>
    public class Servo
    {
        //サーボステータス
        public float Position { get; private set; }    //現在位置
        public float Speed { get; private set; }       //現在速度
        public float Power { get; set; } = 0.0f;        //現在モーター出力

        //内部パラメータ
        float Diff { get; set; }                       //加速度補正
        float ControlAcc { get; set; }                 //目標加速度
        float ResultAcc { get; set; }                  //実測加速度

        //設定値
        const float LimitTorque = 50.0f;                //トルク制限値
        const float TargetSpeed = 3.0f;                 //目標速度
        const float TargetAcc = 1.0f;                   //目標加速度(P)
        const float TargetDec = 1.3f;                   //目標減速度(P)
        const float IGain = 2.0f;                       //Iゲイン
        const float DGain　= 3.0f;                      //Dゲイン

        //固有特性
        const float MotorGain = 1.2f;                   //最大トルク出力値

        /// <summary>
        /// 位置制御
        /// </summary>
        /// <param name="goalPosition"></param>
        public void ControlPosition(float goalPosition)
        {
            var restPosition = goalPosition - Position;     //残り移動量
            ControlSpeed(restPosition);
        }

        /// <summary>
        /// 速度制御(PTP動作を実現)
        /// X軸：時間軸、Y軸：目標速度な台形となるような速度を調整する
        /// 
        ///         ／￣￣＼
        ///       ／        ＼
        ///     ／            ＼
        ///   ／                ＼
        /// ／                    ＼
        /// |  加速区間   |減速区間|
        ///     
        /// 加速区間：目標速度に到達するように目標加速値で加速する
        /// 減速区間：目標速度が現在速度 - 目標減速値となるように減速する
        /// </summary>
        /// <param name="targetSpeed"></param>
        public void ControlSpeed(float restPosition)
        {
            //PTP動作(時間軸Xを基準として加減速を行う)
            var rest = Math.Abs(restPosition);                      //残り移動量(絶対値)
            var hugo2 = (0 <= restPosition) ? +1.0 : -1.0;          //移動したい方向
            var MX = TargetSpeed / TargetDec;                       //減速開始位置X
            var ZX = Math.Sqrt(2 * rest / TargetDec);               //現在位置X  (X+1)*Y/2 = Rest, Y=AX, A=Dec -> X≒√(2*Rest/Dec)

            if ((MX + 1) < ZX)
            {
                //加速区間(速度制限付き、加速カーブは加速上限で表現)
                var spd = hugo2 * Math.Min(TargetAcc, TargetSpeed - Math.Abs(Speed));
                ControlTorque((float)spd);
            }
            else if (TargetDec < rest)
            {
                //減速区間(1つ先の目標速度になるようブレーキをかける)
                var targetspeed = hugo2 * (ZX - 1.0) * TargetDec;
                var spd = targetspeed - Speed;
                ControlTorque((float)spd);
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
        public void ControlTorque(float targetAcc)
        {
            ControlAcc = targetAcc;             //目標加速度（この値は補正値Diffの計算用に使用する）
            Power += targetAcc + Diff;
            //トルク出力制限
            if (+LimitTorque < Power) Power = LimitTorque;
            if (-LimitTorque > Power) Power = -LimitTorque;
        }

        void ControlServo(int current, float power)
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
        float GetFieldPower()
        {
            var random1 = (50 - Position) / 23;
            var random2 = -Speed / 13;
            var random3 = (rnd.Next(0, 200) - 100) / 10000.0;   //エンコーダ誤差はモーター１Tickの１００分の１まで
            //random1 = random2 = 0.0;
            random3 = 0;
            return (float)(random1 + random2 + random3);
        }

        /// <summary>
        /// フィードバック
        /// </summary>
        /// <param name="nextPosition"></param>
        void FeedBack(float nextPosition)
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
