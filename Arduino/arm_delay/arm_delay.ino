#include <Braccio.h>
#include <Servo.h>
Servo base;
Servo shoulder;
Servo elbow;
Servo wrist_ver;
Servo wrist_rot;
Servo gripper;
void setup() {
  Braccio.begin();  //Braccioを初期化
  Serial.begin( 9600 );  //シリアルボードを初期化
}
void loop() {
  //変数宣言
  int inkey;  //入力文字格納用
  //読み込み情報の有無確認
  if( Serial.available() >0 ) {
    //シリアルから値読み込み
    inkey = Serial.read();
    //入力文字の判定
    if (inkey == '8' ){
      //(step delay  M1,  M2,  M3,  M4,  M5,  M6);
      Braccio.ServoMovement(20, 90, 90, 90, 90, 90, 70);
      Serial.print("閉じる");
    }
    if (inkey == '7'){
      //(step delay  M1,  M2,  M3,  M4,  M5,  M6);
      Braccio.ServoMovement(20, 90, 90, 90, 90, 90, 20);
    }
    if (inkey == '11' ){
      base.write(70);
      Serial.print("M1曲げる");
    }
    if (inkey == '12' ){
      base.write(90);
      Serial.print("M1戻す");
    }
    if (inkey == '1' ){
      delay( 100 );  //100ms停止
      shoulder.write(45);
      Serial.print("M2曲げる");
    }
    if (inkey == '2' ){
      delay( 100 );  //100ms停止
      shoulder.write(90);
      Serial.print("M2戻す");
    }
    if (inkey == '1' ){
      delay( 100 );  //100ms停止
      elbow.write(45);
      Serial.print("M3曲げる");
    }
    if (inkey == '2' ){
      delay( 100 );  //100ms停止
      elbow.write(90);
      Serial.print("M3戻す");
    }
    if (inkey == '41' ){
      wrist_ver.write(70);
      Serial.print("M4曲げる");
    }
    if (inkey == '42' ){
      wrist_ver.write(70);
      Serial.print("M4戻す");
    }
    if (inkey == '3' ){
      delay( 100 );  //100ms停止
      wrist_rot.write(135);
      Serial.print("M5曲げる");
    }
    if (inkey == '4' ){
      delay( 100 );  //100ms停止
      wrist_rot.write(90);
      Serial.print("M5戻す");
    }
    if (inkey == '5' ){
      delay( 100);  //100ms停止
      gripper.write(100);
      Serial.print("M6曲げる");
    }
    if (inkey == '6' ){
      delay( 110 );  //100ms停止
      gripper.write(0);
      Serial.print("M6戻す");
    }
  }
  delay( 20 );  //20ms停止
}
