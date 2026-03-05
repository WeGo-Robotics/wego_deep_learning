# Deep Learning Package (v2.0)

본 패키지는 딥러닝 기반 이미지 포인트 추론과 YOLO 객체 인식을 결합한 ROS2 자율주행 제어 시스템입니다. 
라벨링 데이터의 노이즈를 상쇄하는 제어 필터링과 확장된 객체 인식 클래스를 통해 더욱 안정적인 주행 성능을 제공합니다.

---

## 주요 업데이트 사항

### 1. 제어 안정성 최적화

사람이 수동으로 라벨링한 데이터셋의 오차를 극복하기 위해 데드존 및 EMA 기반의 제어 로직을 도입했습니다.

* 데드존 설정
  * $\pm 0.8$ 미만의 미세한 변화는 무시하여 직선 주행 시 조향 떨림을 방지합니다.
* Smoothing 설정
  * 급격한 조향 변화 시 이전 제어값과 현재값을 $5:5$ 비율로 혼합하여 '울컥거림' 현상을 개선했습니다.

### 2. 환경 적응력 및 객체 인식 확장

다양한 도로 상황에 대응하기 위해 학습 모델과 인식 클래스를 대폭 확장했습니다.

* 노면 대응
  * 방지턱 등 불규칙한 노면 환경에서도 추론 점이 튀지 않도록 주행 모델 성능을 강화했습니다.
 
* 인식 클래스 확장
  * 기존: `slow_down`(서행), `speed_up`(속행), `turn_right`(우회전)
  * 추가: `pedestrian` (인식 시 즉시 정지 후 주행), `traffic_light` (신호등 색상별 제어)

---

## 패키지 업데이트 및 빌드 가이드

기존 wego_ws 내의 구버전 패키지를 제거하고 최신 환경을 구축하는 순서입니다. 터미널을 열고 아래 명령어를 순차적으로 실행하세요.

1. 기존 패키지 제거
    ```bash
   $ cd ~/wego_ws/src
   $ rm -rf dl_ros2_application dl_ros2_msgs
    ```
3. 신규 패키지 다운로드 (Git Clone)
    ```bash
   $ cd ~/wego_ws/src
   $ git clone https://github.com/WeGo-Robotics/wego_deep_learning.git
   $ mv wego_deep_learning/dl_ros2_application/ ~/wego_ws/src
   $ mv wego_deep_learning/dl_ros2_msgs/ ~/wego_ws/src
    ```
5. 워크스페이스 빌드
   ```bash
   $ cd ~/wego_ws
   $ colcon build --packages-select dl_ros2_application dl_ros2_msgs 
    ```
6. 워크스페이스 클린 빌드 (전체 초기화 및 안정성 확보)
   ```bash
   $ cd ~/wego_ws
   $ rm -rf build/ install/ log/
   $ colcon build --parallel-workers 2
    ```
   
---

## 부가적인 업데이트 사항

### 1. 시스템 경로 고정

실행 환경에 따른 경로 오차를 방지하기 위해서 데이터셋 저장/라벨 경로와 `launch` 파일에서 모델(`pt`, `onnx`, `engine`) 및 설정 파일을 호출할 때 절대 경로를 참조합니다. 
모든 경로는 `wego_ws` 기준으로 고정되어 있습니다.

* 데이터셋 저장 경로
    ```text
    ├── TF Dataset:   /home/wego/wego_ws/src/dataset/dl_images
    └── YOLO Dataset: /home/wego/wego_ws/src/dataset/yolo_images/images/train
                      /home/wego/wego_ws/src/dataset/yolo_images/images/val
    ```
* YOLO 라벨러 경로
    ```text
    ├── Image Dir:  /home/wego/wego_ws/src/dataset/yolo_images/images/train
    └── Output Dir: /home/wego/wego_ws/src/dataset/yolo_images
    ```
* Model Weights 경로 (.engine)
    ```text
    ├── Drive Model:  .../dl_ros2_application/weight/dl_drive.engine
    └── Object Model: .../dl_ros2_application/weight/best.engine
    ```

### 2. 디버깅 및 시각화 기능 개선

객체 인식 조건 충족 여부를 직관적으로 확인할 수 있도록 디버깅 기능을 강화했습니다.

* 빨간색
  * 객체가 감지되었으나 설정된 `size_threshold`를 만족하지 못해 제어 로직이 활성화되지 않은 상태입니다.
* 초록색
  * 객체 면적이 임계값을 초과하여 해당 제어 플래그(예: 정지, 감속 등)가 활성 상태임을 나타냅니다.

---

## 제어 파라미터 업데이트 사항  

다음 파라미터를 통해 주행 특성을 조정할 수 있습니다.

| 파라미터 | 기본값 | 설명 |
| --- | --- | --- |
| `prev_gain` | `0.7` | 과거 제어값 유지 비중 |
| `current_gain` | `0.3` | 신규 추론값 반영 비중 |
| `threshold_max` | `1.0` | 스무딩 필터 적용 상한선 |
| `threshold_min` | `0.8` | 조향 무시 범위 (데드존) |
| `deceleration_ratio` | `0.45` | 조향각에 따른 동적 감속 비율 |

---

## YOLO 객체 인식 파라미터 업데이트 사항  

다음 파라미터를 통해 객체 인식 특성을 조정할 수 있습니다.

| 파라미터 | 설정값 | 상세 설명 |
| --- | --- | --- |
| `yellow_light_factor` | `0.5` | 신호등이 노란불일 때, 서행 비중 |
| `green_light_factor` | `1.0` | 신호등이 초록불일 때, 주행 비중 |
| `red_light_duration_sec` | `1.0` | 신호등이 빨간불일 때, 정지 상태를 유지하는 최소 시간 |
| `yellow_light_duration_sec` | `1.0` | 신호등이 노란불일 때, 서행 상태를 유지하는 최소 시간 |
| `green_light_duration_sec` | `1.0` | 신호등이 초록불일 때, 주행 상태를 유지하는 최소 시간 |
| `pedestrian_duration_sec` | `3.0` | 보행자 표지판이 시야에서 사라진 후에도 일시 정지를 유지하는 시간 (보행자가 횡단보도를 건너는 최소 시간을 보장) |
| `pedestrian_pass_by_duration_sec` | `6.0` | 보행자 상황 발생 후, 다음 보행자 이벤트를 재활성화하기까지의 대기 시간 (동일 보행자에 대해 중복 멈춤이 일어나는 것을 방지) |
| `red_light_size_threshold` | `7000.0` | 빨간불 인식 기준 면적 (정지선 근처 도달 시 정지 유도) |
| `yellow_light_size_threshold` | `2700.0` | 노란불 인식 기준 면적 |
| `green_light_size_threshold` | `7000.0` | 초록불 인식 기준 면적 |
| `pedestrian_size_threshold` | `4000.0` | 보행자 표지판 인식 기준 면적|

---

## 참고자료
Limo Wiki
https://goofy-pleasure-a84.notion.site/Limo-Wiki-a6aa65b627cb40019a82d469dc5ae69d
