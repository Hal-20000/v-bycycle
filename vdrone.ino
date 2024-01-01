/*
 *  camera_recognition.ino - AI判定によるカメラ撮影画像のドア開閉判別をするサンプルプログラム
 *  Copyright 2023 CRESCO LTD.
 */
/**
 * 説明
 *   本プログラムは、シリアルモニタが接続されるのを待ち、その後、
 *   1秒おきにSpresenseカメラボードを使って撮影し、その撮影画像をAIモデルを用いてドア開閉判別をします。
 *   ドア開閉判別の推論結果は、シリアルモニタで確認することができます。
 *   また、ファイル無しエラー、DNNRT開始エラー、カメラ処初期化エラー、カメラ設定エラーが発生した場合、
 *   シリアルモニタへログを出力し、停止します。
 * 
 * 動作環境
 *   Arduino IDEを利用するSpresense Arduino 開発環境(2.3.0)上で利用できます。
 *   Spresenseメインボードに合わせて以下の機材が必要となります。
 *     ・Spresense拡張ボード
 *     ・Spresenseカメラボード
 *     ・SDカード
 * 
 * 利用方法
 *   1. NNC（Neural Network Console）を用いてドア開閉判別に利用するAIモデルを作成します。
 *   2. AIモデルをSDカード内に格納し、SDカードを拡張ボードへ装着します。
 *      格納する際にファイル名は「network.nnb」としてください。
 *   3. 本プログラムをArduino IDEで開き、Spresenseに書き込みます。
 *   4. Arduino IDEと接続し、Arduino IDEの上部メニューから「ツール」->「シリアルモニタ」を選択し、シリアルモニタを起動します。
 *      ・推論結果は"[recognition]"から始まる出力で確認できます。
 * 
 * 補足事項
 *   本プログラムは、AIモデルの入力サイズ(横80縦60グレースケール形式)に適用させるため、カメラ撮影画像を以下のように変換しています。
 *     1. 横320縦240のサイズ、YUV形式で撮影します。
 *        ライブラリで指定可能な最小サイズでの撮影となります。
 *     2. 撮影画像を1/4に縮小します。
 *        ライブラリを使った縮小比率は2のべき乗という制限に合わせたものとなります。
 *     3. グレースケール形式へ変換します。
 *   ここまでで、横80縦60のサイズの画像ができますので、AIモデルの入力とします。
 * 
 * 製品情報
 *   Spresenseカメラボード：CXD5602PWBCAM1
 *   https://developer.sony.com/ja/develop/spresense/specifications
 * 
 * 参考情報
 *   NNC（Neural Network Console）については下記を参照してください。
 *     https://dl.sony.com/ja/
 */


#include <SDHCI.h>
#include <DNNRT.h>
#include <Camera.h>
#include <EltresAddonBoard.h>


// AIモデルへの入力サイズ
// #define INPUT_WIDTH (80)
// #define INPUT_HEIGHT (60)
#define INPUT_WIDTH (56)
#define INPUT_HEIGHT (56)


// 拡縮比率（２のべき乗）
#define INPUT_RESIZE_RATIO (4)
// 撮影サイズ
#define PICTURE_WIDTH (CAM_IMGSIZE_QVGA_H)
#define PICTURE_HEIGTH (CAM_IMGSIZE_QVGA_V)
// 切り取りサイズ
#define PICTURE_CLIP_WIDTH (INPUT_WIDTH * INPUT_RESIZE_RATIO)
#define PICTURE_CLIP_HEIGHT (INPUT_HEIGHT * INPUT_RESIZE_RATIO)

//　DNNRTクラスのインスタンス
DNNRT dnnrt;
// SDカードクラスのインスタンス
SDClass SD;
// AIモデルの入力データ用領域
DNNVariable dnn_input(INPUT_WIDTH *INPUT_HEIGHT);


// PIN定義：LED(プログラム状態)
#define LED_RUN PIN_LED0
// PIN定義：LED(GNSS電波状態)
#define LED_GNSS PIN_LED1
// PIN定義：LED(ELTRES状態)
#define LED_SND PIN_LED2
// PIN定義：LED(エラー状態)
#define LED_ERR PIN_LED3

// プログラム内部状態：初期状態
#define PROGRAM_STS_INIT (0)
// プログラム内部状態：起動中
#define PROGRAM_STS_RUNNING (1)
// プログラム内部状態：終了
#define PROGRAM_STS_STOPPED (3)

// プログラム内部状態
int program_sts = PROGRAM_STS_INIT;
// GNSS電波受信タイムアウト（GNSS受信エラー）発生フラグ
bool gnss_recevie_timeout = false;
// 点滅処理で最後に変更した時間
uint64_t last_change_blink_time = 0;
// イベント通知での送信直前通知（5秒前）受信フラグ
bool event_send_ready = false;
// ペイロードデータ格納場所
uint8_t payload[16];

// // VL53L0X制御用インスタンス
// static VL53L0X distance_sensor;
// 最新値（距離）
// uint16_t last_distance;

// 最新のGGA情報
eltres_board_gga_info last_gga_info;

/**
 * @brief イベント通知受信コールバック
 * @param event イベント種別
 */
void eltres_event_cb(eltres_board_event event) {
  switch (event) {
    case ELTRES_BOARD_EVT_GNSS_TMOUT:
      // GNSS電波受信タイムアウト
      Serial.println("gnss wait timeout error.");
      gnss_recevie_timeout = true;
      break;
    case ELTRES_BOARD_EVT_IDLE:
      // アイドル状態
      Serial.println("waiting sending timings.");
      digitalWrite(LED_SND, LOW);
      break;
    case ELTRES_BOARD_EVT_SEND_READY:
      // 送信直前通知（5秒前）
      Serial.println("Shortly before sending, so setup payload if need.");
      event_send_ready = true;
      break;
    case ELTRES_BOARD_EVT_SENDING:
      // 送信開始
      Serial.println("start sending.");
      digitalWrite(LED_SND, HIGH);
      break;
    case ELTRES_BOARD_EVT_GNSS_UNRECEIVE:
      // GNSS電波未受信
      Serial.println("gnss wave has not been received.");
      digitalWrite(LED_GNSS, LOW);
      break;
    case ELTRES_BOARD_EVT_GNSS_RECEIVE:
      // GNSS電波受信
      Serial.println("gnss wave has been received.");
      digitalWrite(LED_GNSS, HIGH);
      gnss_recevie_timeout = false;
      break;
    case ELTRES_BOARD_EVT_FAULT:
      // 内部エラー発生
      Serial.println("internal error.");
      break;
  }
}


/**
 * @brief GGA情報受信コールバック
 * @param gga_info GGA情報のポインタ
 */
void gga_event_cb(const eltres_board_gga_info *gga_info) {
  Serial.print("[gga]");
  last_gga_info = *gga_info;
  if (gga_info->m_pos_status) {
    // 測位状態
    // GGA情報をシリアルモニタへ出力
    Serial.print("utc: ");
    Serial.println((const char *)gga_info->m_utc);
    Serial.print("lat: ");
    Serial.print((const char *)gga_info->m_n_s);
    Serial.print((const char *)gga_info->m_lat);
    Serial.print(", lon: ");
    Serial.print((const char *)gga_info->m_e_w);
    Serial.println((const char *)gga_info->m_lon);
    Serial.print("pos_status: ");
    Serial.print(gga_info->m_pos_status);
    Serial.print(", sat_used: ");
    Serial.println(gga_info->m_sat_used);
    Serial.print("hdop: ");
    Serial.print(gga_info->m_hdop);
    Serial.print(", height: ");
    Serial.print(gga_info->m_height);
    Serial.print(" m, geoid: ");
    Serial.print(gga_info->m_geoid);
    Serial.println(" m");
  } else {
    // 非測位状態
    // "invalid data"をシリアルモニタへ出力
    Serial.println("invalid data.");
  }
}

long flag = 0;

/**
 * @brief セットアップ処理
 */
void setup() {

  Serial.begin(115200);
  randomSeed(0);
  // LED初期設定
  pinMode(LED_RUN, OUTPUT);
  digitalWrite(LED_RUN, HIGH);
  pinMode(LED_GNSS, OUTPUT);
  digitalWrite(LED_GNSS, LOW);
  pinMode(LED_SND, OUTPUT);
  digitalWrite(LED_SND, LOW);
  pinMode(LED_ERR, OUTPUT);
  digitalWrite(LED_ERR, LOW);

  // ELTRES起動処理
  // eltres_board_result ret = EltresAddonBoard.begin(ELTRES_BOARD_SEND_MODE_1MIN, eltres_event_cb, NULL);
  eltres_board_result ret_elt = EltresAddonBoard.begin(ELTRES_BOARD_SEND_MODE_1MIN, eltres_event_cb, gga_event_cb);
  if (ret_elt != ELTRES_BOARD_RESULT_OK) {
    // ELTRESエラー発生
    digitalWrite(LED_RUN, LOW);
    digitalWrite(LED_ERR, HIGH);
    program_sts = PROGRAM_STS_STOPPED;
    Serial.print("cannot start eltres board (");
    Serial.print(ret_elt);
    Serial.println(").");
    return;
  }

  int ret;
  CamErr cam_err;

  Serial.begin(115200);
  while (!Serial) {
    ;  // シリアルモニタ接続待ち
  }

  // AIモデルファイル読み込み
  File nnbfile = SD.open("network.nnb");
  if (nnbfile == NULL) {
    // ファイル無しエラー
    Serial.print("nnb is not found");
    exit(0);
  }
  // DNNRTライブラリ（AIモデルでの判定を行うライブラリ）の初期設定
  ret = dnnrt.begin(nnbfile);
  if (ret < 0) {
    // DNNRT開始エラー
    Serial.print("DNNRT initialization failure.: ");
    Serial.println(ret);
    exit(0);
  }
  nnbfile.close();

  // カメラの初期設定
  cam_err = theCamera.begin(0);
  if (cam_err != CAM_ERR_SUCCESS) {
    // カメラ開始エラー
    Serial.print("CAMERA initialization failure.: ");
    Serial.println(cam_err);
    exit(0);
  }
  // 撮影パラメタ（サイズ、形式）設定
  cam_err = theCamera.setStillPictureImageFormat(PICTURE_WIDTH, PICTURE_HEIGTH, CAM_IMAGE_PIX_FMT_YUV422);
  if (cam_err != CAM_ERR_SUCCESS) {
    // カメラ設定エラー
    Serial.print("CAMERA set parameters failure.: ");
    Serial.println(cam_err);
    exit(0);
  } else {
    // 正常
    program_sts = PROGRAM_STS_RUNNING;
  }



  // センサ初期設定
  // Wire.begin();
  // distance_sensor.setTimeout(500);
  // if (distance_sensor.init() == false) {
  //   // センサ初期設定エラー
  //   EltresAddonBoard.end();
  //   digitalWrite(LED_RUN, LOW);
  //   digitalWrite(LED_ERR, HIGH);
  //   program_sts = PROGRAM_STS_STOPPED;
  //   Serial.println("cannnot start a distance sensor.");
  //   return;
  // }
  // 測定開始
  // distance_sensor.startContinuous();
  // 正常
  program_sts = PROGRAM_STS_RUNNING;

  Serial.println("続行");
}

/**
 * @brief ループ処理
 */
void loop() {


  Serial.println("Loop in.");
  flag = (random(0, 2));
  Serial.print("Flag is ");
  Serial.println(flag);

  CamErr cam_err;
  CamImage coverted;

  // 1秒待機
  sleep(1);

  // カメラ撮影
  CamImage camImage = theCamera.takePicture();
  if (!camImage.isAvailable()) {
    // 撮影失敗
    Serial.println("CAMERA take picture failure.");
    return;
  } else {
    Serial.println("CAMERA take picture success.");
  }
  // 画像の切り取りと縮小処理
  int lefttop_x = (PICTURE_WIDTH - PICTURE_CLIP_WIDTH) / 2;
  int lefttop_y = (PICTURE_HEIGTH - PICTURE_CLIP_HEIGHT) / 2;
  int rightbottom_x = lefttop_x + PICTURE_CLIP_WIDTH - 1;
  int rightbottom_y = lefttop_y + PICTURE_CLIP_HEIGHT - 1;
  cam_err = camImage.clipAndResizeImageByHW(coverted,
                                            lefttop_x, lefttop_y, rightbottom_x, rightbottom_y,
                                            INPUT_WIDTH, INPUT_HEIGHT);
  if (cam_err != CAM_ERR_SUCCESS) {
    Serial.print("CAMERA resize failure. : ");
    Serial.println(cam_err);
    return;
  }
  // グレースケール形式への変換
  cam_err = coverted.convertPixFormat(CAM_IMAGE_PIX_FMT_GRAY);
  if (cam_err != CAM_ERR_SUCCESS) {
    Serial.print("CAMERA convert format failure. : ");
    Serial.println(cam_err);
    return;
  }

  // AIモデルの入力データ設定
  float* input_data = dnn_input.data();
  uint8_t* camera_buf = coverted.getImgBuff();
  for (int h = 0; h < INPUT_HEIGHT; h++) {
    for (int w = 0; w < INPUT_WIDTH; w++) {
      input_data[h * INPUT_WIDTH + w] = camera_buf[h * INPUT_WIDTH + w] / 255.0;
    }
  }
  dnnrt.inputVariable(dnn_input, 0);
  // 推論実行
  dnnrt.forward();
  // 推論結果取得
  DNNVariable output = dnnrt.outputVariable(0);
  float value = output[0];


  if (value) {

    // 結果表示
    Serial.print("Object detection!!");

    Serial.print(" ( value: ");
    Serial.print(value);
    // if (value < 0.5f) {
    //   Serial.print("close.");
    // } else {
    //   Serial.print("open. ");
    // }
    // Serial.print(" ( value: ");
    // Serial.print(value);
    // Serial.println(")");
  }

  switch (program_sts) {
    case PROGRAM_STS_RUNNING:
      // プログラム内部状態：起動中
      if (gnss_recevie_timeout) {
        // GNSS電波受信タイムアウト（GNSS受信エラー）時の点滅処理
        uint64_t now_time = millis();
        if ((now_time - last_change_blink_time) >= 1000) {
          last_change_blink_time = now_time;
          bool set_value = digitalRead(LED_ERR);
          bool next_value = (set_value == LOW) ? HIGH : LOW;
          digitalWrite(LED_ERR, next_value);
        }
        Serial.println(gnss_recevie_timeout);
      } else {
        digitalWrite(LED_ERR, LOW);
      }

      if (event_send_ready) {
        // 送信直前通知時の処理
        event_send_ready = false;
        setup_payload_detection((int16_t)flag);
        // 送信ペイロードの設定
        EltresAddonBoard.set_payload(payload);
      }

      // 距離センサから測定値を取得し、最新値を更新
      // measure_distance();
      break;

    case PROGRAM_STS_STOPPED:
      // プログラム内部状態：終了
      break;
  }
}



/**
 * @brief ペイロード設定
 * @param flag フラグ
 */
void setup_payload_detection(int16_t flag) {

  uint32_t gnss_time;
  uint32_t utc_time;
  // GNSS時刻(epoch秒)の取得
  EltresAddonBoard.get_gnss_time(&gnss_time);
  // UTC時刻を計算（閏秒補正）
  utc_time = gnss_time - 18;

  // 設定情報をシリアルモニタへ出力
  Serial.println("[setup_payload_detection]");
  Serial.print("detection flag:");
  Serial.print(flag);
  Serial.println();

  // ペイロード領域初期化
  memset(payload, 0x00, sizeof(payload));
  // ペイロード種別[気圧圧力照度距離ペイロード]設定
  payload[0] = 0x8A;

  // 衝突判定flag設定
  payload[1] = (uint8_t)((flag)&0xff);
  //payload[14] = (uint8_t)(distance & 0xff);
  // 時刻(EPOCH秒)設定
  payload[10] = (uint8_t)((utc_time >> 24) & 0xff);
  payload[11] = (uint8_t)((utc_time >> 16) & 0xff);
  payload[12] = (uint8_t)((utc_time >> 8) & 0xff);
  payload[13] = (uint8_t)(utc_time & 0xff);
  // 拡張用領域(0固定)設定
  payload[14] = 0x00;
  // 品質設定
  payload[15] = last_gga_info.m_pos_status;
}
