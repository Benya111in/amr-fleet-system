> 사본. 원본 `/home/CAPYI_AI/research/reports/3D LiDAR SLAM 도입 방안.md` (2026-10-06 05:07 KST, 86 499 B, sha256 6866da6adc401109…), 2026-10-06 복사. 본문은 고치지 않았다.
> 이 저장소의 측정·계산으로 달라진 점(대역 해석, S6 대기 로봇, 0.05 m GT 바닥값, S5 목표 수, 원본 스크립트 복셀 인자)은 [evaluation.md](evaluation.md) §0·§2·§3·§6 에 적었다.
> 근거 노트 원본: `/home/CAPYI_AI/research/research_notes/3D LiDAR SLAM 도입 방안/` (저장소에 넣지 않음).

# 3D SLAM은 RTAB-Map으로 붙이고 조건을 맞춰 잰다

2026년 10월까지 나온 논문, 오픈소스 코드, 제조사 자료, 업계 보도를 종합하면 명세 보너스 2는 다음처럼 출발하는 것이 맞다. 로봇 5대 가운데 1대에만 가상 16채널 회전형 3D LiDAR(수직 ±15°, 1800열, 10 Hz)를 지면 0.90 m 마스트에 단다. 주력 3D SLAM은 ROS 2 Humble 공식 apt로 설치되는 **RTAB-Map**(BSD)으로 한다. RTAB-Map은 Gazebo 점군을 그대로 받고, 기존 휠-EKF 오도메트리를 ICP 초기값으로 쓴다. 복도처럼 구속이 약한 축은 그 초기값으로 채운다. 3D 지도에서 높이 대역을 골라 Nav2용 2D 격자를 만들고, 위치추정 모드에서는 map→odom 또는 글로벌 EKF에 넣을 자세를 낸다. 명세가 이름을 든 **LIO-SAM**(ros2 브랜치)은 별도 컨테이너에서 소스 빌드해 같은 rosbag으로 돌리는 매핑 비교 행으로 둔다. Cartographer 3D는 업스트림이 유지보수 종료를 공지했고, FAST-LIO2는 GPL·Livox 빌드 의존성에 더해 시간 필드가 없으면 가짜 점 시각을 지어낸다. GLIM은 사전지도 위치추정이 없고 GTSAM 4.3 충돌 위험이 있다. 그래서 셋 다 핵심 경로에서 뺀다. 가장 중요한 발견은 비교의 공정성에 관한 것이다. Gazebo IMU의 방위는 잡음 없는 정답이라 LIO-SAM과 RTAB-Map의 IMU 경로에 그대로 넣으면 3D 쪽만 정답 자세를 받는다. 시뮬레이션 휠은 실기보다 1–2자릿수 정확해 휠을 쓰는 방법을 모두 오라클로 만든다. Gazebo 3D 점군은 한 순간에 렌더되어 모션 왜곡이 없다. 따라서 "한 번 기록하고 여러 번 재생"해 입력을 고정하고, 센서(2D/3D) × 휠 사전값 × 휠 품질 × 통로 끝 막힘의 요인 설계로 차원 효과를 따로 떼어 내야 한다. 문헌과 이번 자체 계산을 합친 기대 결과는 이렇다. 정상 조건에서 2D와 3D의 위치 정확도 차이는 반복 간 변동 수준일 가능성이 크다. 현행 2D 위치추정 RMSE(0.9–1.7 cm)와 실험실 3D LIO의 ATE(2.5–3.5 cm)가 이미 같은 자릿수이기 때문이다. 3D의 이득은 세 곳에서 드러난다. 대기 로봇이 좁은 통로 끝을 막을 때(현재 2D는 통로 축 구속을 통로 끝 너머 시야에 기대고 있다), 스캔 평면 위의 돌출 장애물, 0.20 m 단면 밖 구조물의 지도 완전성이다. "3D가 일반적으로 더 정확하다", "3D가 점대칭 별칭을 푼다", "deskew 효과가 있다", "5대 동시 3D 운용이 된다"는 측정 없이 주장할 수 없다.

## 3D가 이기는 조건은 좁고, 우리 월드에서는 막힌 통로 끝이다

같은 조건에서 2D SLAM과 3D SLAM을 정량 비교한 동료심사 연구는 드물다. 있는 연구도 차원을 다른 변수와 섞는다. 자주 인용되는 Sobczak 등(Sci. Rep. 2022)은 제목부터 "best hardware configuration **for 2D SLAM**"이다. 모든 구성의 지도와 궤적을 Cartographer 2D로 만들었으므로 3D LiDAR(VLP-16)도 2D 알고리즘의 입력으로만 쓰였다. 25 m가 넘는 복도에서 3D LiDAR + IMU가 더 나았지만, 저자는 원인으로 2D LiDAR의 사거리 12 m를 들었다. 3D LiDAR는 UV 램프 위에, 2D LiDAR는 차체 하단에 달려 장착 높이도 달랐다. 시뮬레이션 IMU의 자세 쿼터니언은 시뮬레이터에서 바로 읽은 정답이었다 ([Sobczak et al., Sci. Rep. 2022](https://doi.org/10.1038/s41598-022-22938-y)). ROS 1 move_base로 2D·3D 지도를 비교한 COINS 2023 논문은 초록에서 3D 우위를 말하지만 수치는 공개 범위에서 확인되지 않는다 ([Chen et al., COINS 2023](https://doi.org/10.1109/COINS57856.2023.10189271)). 정리하면 문헌의 "3D가 낫다"는 사거리·장착 높이·알고리즘 효과의 합이다. 보너스 2 보고서에서 "차원 효과"라고 부르려면 이 셋을 통제해야 한다.

창고에 대한 공개 정량 근거는 사실상 Kinematic-ICP(ICRA 2025) 하나다. 이 논문은 Dexory 차동구동 로봇(32빔 Bpearl, 10 Hz)으로 실제 창고 세 곳(0.35/2.3/9.45 ha)을 달렸다. LiDAR 단독 KISS-ICP의 ATE는 **7.36/54.70/23.50 m**로 무너졌다. 휠을 초기값으로 주고 6-DoF로 추정한 "WO + 3D"는 5.40/28.71/17.52 m였다. 같은 3D 점군에 평면 운동 제약을 건 "WO + 2D"는 1.01/6.48/4.11 m였다. 휠 정규화와 유니사이클 제약을 넣은 Kinematic-ICP는 **0.26/4.44/4.42 m**였다 ([Guadagnino et al., arXiv 2410.10277](https://arxiv.org/abs/2410.10277)). 해석에는 세 가지 주의가 필요하다. 첫째, "WO + 2D"의 "2D"는 2D LiDAR가 아니라 같은 3D LiDAR에 SE(2) 제약을 건 변형이다. 그러니 이 표는 "2D 센서가 3D 센서보다 낫다"의 근거가 아니다. 둘째, 창고의 기준 궤적은 토털스테이션이 아니라 Cartographer 결과다. 셋째, 공저자 두 명이 Dexory 소속이라 기업 이해관계가 있다. 이 표가 실제로 보여 주는 것은 "LiDAR 단독 오도메트리는 반복 랙 통로에서 모호해 실패하고, 정확도는 휠 결합과 평면 제약이 정한다"는 점이다. 같은 논문의 정규화 절제에서 소형 창고는 휠을 가장 강하게 믿은 고정 β=0.01이 ATE 0.20 m로 가장 좋았고, 정규화를 빼면 1.59 m였다.

사전지도 위치추정 쪽 근거도 차이가 작다는 쪽을 가리킨다. KISS-SLAM 논문은 3D 점유격자를 높이 0.1–0.2 m로 잘라 2D 지도를 만들고 2D LiDAR 로봇의 MCL을 돌렸다. 저자 평가는 GMapping 지도와 "유의한 차이 없음"이다. 하지만 동적 시퀀스 두 개에서는 GMapping 지도가 ATE(5.71 대 9.57 cm, 7.43 대 10.42 cm)에서 앞섰다. 동적 시퀀스 1의 성공률은 두 지도 모두 90%였고, 실험 장소는 창고가 아닌 사무실이다 ([Guadagnino, Mersch et al., arXiv 2503.12660](https://arxiv.org/abs/2503.12660)). OptiTrack 기준 실내 벤치마크 IILABS 3D에서 VLP-16의 LIO-SAM 계열(LIORF) ATE는 2.5–3.5 cm였다 ([IILABS 3D, IEEE Access 2025](https://doi.org/10.1109/ACCESS.2025.3643753)). 우리 현행 2D 스택의 값은 같은 자릿수다. 지도 ADNN₀는 2.59 cm이고, 위치추정 RMSE는 정지 1.52 / 직선 1.74 / 회전 0.90 cm다. 다만 정지 최대 오차 4.23 cm는 명세 3 cm를 넘는다. 원인은 도크 쪽 지도의 국소 어긋남(2.6–3.2 cm)이었고, RTF 중앙값 0.27의 부하 큰 호스트에서 잰 값이다(`docs/algorithms/slam.md` §6.1·§6.3). 따라서 평균 정확도 차이는 반복 간 변동과 같은 크기일 가능성이 크다. 짝지은 반복 측정 없이 우열을 말할 수 없다.

업계 관행은 "2D 체인을 유지하고 3D를 덧붙이는" 형태다. MiR1200 Pallet Jack 공식 사양서는 안전 레이저 스캐너 3대가 "360° personnel detection"을, 3D LiDAR 1대가 "obstacle detection"을, 3D 카메라 5대가 팔레트·장애물을 맡는다고 적는다 ([MiR1200 사양서 v2.101](https://www.konicaminolta.com.au/wp-content/uploads/2025/06/MiR1200-Pallet-Jack-Specifications-2.101.pdf)). OTTO Lifter는 SICK microScan3 안전 스캐너 3대와 RealSense 5대를 단다 ([Vision Systems Design](https://www.vision-systems.com/astech-autonomy/article/14270081/otto-motors-launches-autonomous-forklift)). Seegrid의 3D LiDAR도 주행 경로와 팔레트 적치 위치 감시용이다 ([Seegrid 2025-06](https://seegrid.com/news/seegrid-elevates-amr-safety-with-advanced-layered-obstruction-detection/)). 3D LiDAR를 주 위치추정에 쓰는 쪽은 Third Wave Automation, Thoro.ai, 트위니, 유진로봇 같은 신흥 업체다 ([The Robot Report 2024-10-30](https://www.therobotreport.com/third-wave-automation-picks-series-c-funding-automated-forklifts/), [Hesai MODEX 2026](https://www.hesaitech.com/hesai-powers-safety-certified-ai-autonomy-for-smart-logistics-with-3d-lidar-at-modex-2026/), [로봇신문 2026-03-04](https://www.irobotnews.com/news/articleView.html?idxno=45189)). 이들 가운데 위치 정확도 수치를 공개한 곳은 이번 조사에서 찾지 못했다. 트위니 대표는 물류센터 공간 변화율이 40–70%라 표준 알고리즘 AMR은 "10분도 버티지 못한다"고 말했지만, 근거는 제시되지 않았다 ([한국경제 2026-08-05](https://www.hankyung.com/article/2026080569801)). 안전 규격도 같은 분업을 강제한다. ISO 3691-4는 사람 감지와 제동에 PLr d를 요구한다(2020판 TÜV 해설 기준) ([TÜV Rheinland 백서](https://www.tuv.com/content-media-files/master-content/services/industrial-services/pdf/tuv-rheinland-automatic-guided-vehicles-whitepaper-en_neu.pdf)). 제3자 인증이 확인된 광학식 3D LiDAR의 최고 등급은 SICK multiScan100-S의 **PL b**다 ([WorldCargo News 2025-06](https://www.worldcargonews.com/ict-automation/2025/06/sick-launches-safety-certified-3d-lidar-sensor/)). PL d는 Ouster REV8과 RoboSense Safety Airy(예약판매)가 "설계 목표"로 내걸었을 뿐이다 ([Ouster REV8](https://ouster.com/rev8), [RoboSense Safety Airy](https://www.robosense.ai/en/IncrementalComponents/SafetyAiry)). 비광학식인 Sonair 3D 초음파 센서는 exida 인증 PL d를 발표했다 ([Sonair 2026-06-30](https://www.sonair.com/journal/worlds-first-safety-certified-3d-ultrasonic-sensor)). 로봇용 LiDAR 출하 급증도 창고 AMR 덕분이 아니다. Hesai의 2025년 "Robotics" 출하 239,273대는 정의상 로보택시·배송로봇·AGV를 함께 담은 범주이고, 창고 AMR 물량은 따로 밝히지 않는다. 대신 잔디깎이 수주 잔고 1,000만 대 이상을 명시한다 ([Hesai SEC 6-K 2026-03-24](https://www.sec.gov/Archives/edgar/data/1861737/000110465926033591/tm269592d1_ex99-1.htm)).

우리 월드에서 3D가 실제로 이기는 조건은 이번 조사의 자체 계산으로 좁혔다. 월드 생성 스크립트의 정답 기하에 광선을 쏘아 단일 스캔의 점-대-평면 Fisher 정보를 셌다. 위치는 4 m 길이 좁은 통로(순폭 0.60 m)의 중앙이고, 잡음은 σ=0.03 m, 대응은 완전하다고 가정했다. 스크립트는 `/home/CAPYI_AI/research/artifacts/3d_slam/localizability2.py`이다(2026-10-06 연구 산출물 폴더로 옮김). 결과는 처음 예상과 달랐다.

| 센서·장착(광학 중심 높이) | 통로 끝 열림: 통로 축 구속 점 / 병진 정보 λmin/λmax | 4 m 사거리 상한(통로 끝 막힘 대용) |
|---|---|---|
| 2D 720빔 @0.20 m (현행) | 70 / 0.11 | **0 / ≈0 (완전 퇴화)** |
| 16ch ±15° @0.45 m | 2,158 / 0.19 | 956 / 0.08 |
| 16ch ±15° @0.85 m | 4,839 / 0.75 | 1,616 / 0.28 |
| 16ch ±15° @1.20 m | 3,861 / 0.72 | 728 / 0.15 |
| 16ch ±15° @0.85 m, 15° 하향 | 4,216 / 0.65 | 1,210 / 0.20 |
| Mid-360형(−7°~52°) @0.85 m | 4,460 / 0.66 | 1,116 / 0.18 |
| OS0-64형(±45°) @0.85 m | 4,898 / 0.38 | 1,712 / 0.14 |

현재 월드의 좁은 통로는 양끝이 열려 있다. 그래서 2D도 통로 끝 너머의 벽과 C열 랙을 보는 빔(중앙에서 앞뒤 각 ±8.5°, 약 34빔씩)으로 통로 축을 약하게나마 구속한다. 3D는 데크 위 상자와 0.08 m 기둥 덕분에 어느 장착에서든 강하게 구속된다. 통로 끝을 막으면 2D만 완전히 퇴화한다. 이 조건은 우리 시스템에서 실제로 생긴다. traffic_manager가 좁은 통로에 용량 1 토큰을 걸기 때문에, 대기 로봇이 통로 입구에 서서 2D의 끝 시야를 가린다. 그러므로 **"통로 끝에 대기 로봇이나 작업자를 세운" 시나리오가 3D의 이점을 보일 가장 현실적인 실험**이다. 같은 계산에서 3D 센서 종류 사이의 차이는 2D–3D 차이보다 훨씬 작았고, 15° 하향 기울임은 벽 시야를 잃어 오히려 나빴다. 이 수치는 완전 대응을 가정한 하한이라 절대값이 아니라 순위 비교에만 써야 한다. 상자 무작위 자세와 동적 행위자는 계산에서 뺐고, Gazebo gpu_lidar 실제 출력으로 교차 확인하기 전이다.

점대칭 별칭은 3D로 풀린다고 기대하면 안 된다. 회전 불변 기술자는 자세 p와 그 180° 거울 자세 p'에서 같은 값을 낸다. 3D는 2D 스캔 평면 위쪽에 비대칭 구조(도크 개구부, 충전소, 표지판, 랙 상단 적재물)가 보일 때만 대칭을 깬다. 2026년 프리프린트 "Move First, Commit Later"는 반복 실내의 주된 실패를 "확신에 찬 별칭(confident aliasing)"으로 정식화했다. 다만 이 방법의 격자 검출기는 병진 대칭만 다루고, 회전·거울 대칭은 향후 과제로 남겼다. 실험도 건물 한 곳, 프런트엔드 하나의 6개 구성이다 ([Zhang et al., arXiv 2607.17103](https://arxiv.org/abs/2607.17103)). lidar_localization_ros2의 운용 문서에는 임계값 2.0에서 검증 루브릭을 통과했지만 정답 기준 25–28 m 떨어진 별칭 오확정이 기록돼 있다(실외 경로) ([global_localization.md](https://github.com/rsasaki0109/lidar_localization_ros2/blob/main/docs/global_localization.md)). 따라서 보고서에서 "3D가 별칭을 푸는가"는 가설로 두고 검증해야 한다. 가장 설득력 있는 근거는 사전 분석이다. 정답 3D 점군에서 격자 위 각 자세 p와 거울 자세 p'의 가상 스캔을 만들고 정합 점수 차를 지도로 그리면, "3D가 대칭을 깨는 영역"과 "여전히 모호한 영역"이 실험 전에 드러난다.

## 가상 16채널 라이다를 0.90 m 마스트에 단다

### 실기 대응 표기는 "16채널 회전형"으로 하고, 비반복 패턴은 흉내 내지 않는다

시뮬레이션 센서는 실물 구매 없이 "어떤 급의 센서를 모델링했는가"만 정당화하면 된다. VLP-16은 단종됐다. Velodyne를 인수한 Ouster의 공지를 대리점이 옮긴 바에 따르면 수리 지원은 2026-06-30에, 기술 지원은 2026년 말에 끝난다 ([Digiflec 2024-09](https://digiflec.com/important-update-for-ouster-velodyne-vlp-16-users/)). Ouster는 Rev8 세대로 넘어갔다. OS1은 수직 44°, OS0은 90°이고, 내장 IMU는 여전히 6축이며 가격은 견적제다 ([OS1 Rev8 데이터시트](https://data.ouster.io/downloads/datasheets/datasheet-rev8-v4p0-os1.pdf), [OS0 Rev8 데이터시트](https://data.ouster.io/downloads/datasheets/datasheet-rev8-v4p0-os0.pdf)). 시장은 1,000달러 안팎의 반구형·광시야 제품으로 옮겨 갔다. Livox Mid-360은 2023년 시험 구매가가 749달러였고 2026-09 보도 기준 정가는 979달러(관세에 따라 변동)다 ([Livox 출시 보도자료](https://www.livoxtech.com/news/mid360_launch), [DroneXL 2026-09-21](https://dronexl.co/2026/09/21/livox-mid-360l-lidar-half-point-rate-robots/)). RoboSense Airy 96빔은 999달러 ([RoboSense 스토어](https://store.robosense.ai/products/airy)), Hesai JT16은 판매처 등재가 950달러 ([Cratus](https://www.cratustech.com/product/hesai-jt16/)), Unitree L2는 419달러다 ([Unitree](https://shop.unitree.com/products/unitree-4d-lidar-l2)). 반면 기계식 16채널 Hesai XT16은 6,650달러에 품절이다 ([RoboStore](https://robostore.com/products/hesai-xt16-3d-lidar)).

Mid-360의 비반복 패턴은 표준 gpu_lidar로 재현할 수 없다. Humble+Fortress용 공개 구현은 두 개 있다. 그러나 DHA-Tappuri/livox_ignition은 라이선스 파일이 없고, jongbob1918/livox_mid360_gz_sim_ros2는 "Proprietary, LK Robotics"라 재사용할 권리가 없다 ([livox_ignition](https://github.com/DHA-Tappuri/livox_ignition), [livox_mid360_gz_sim_ros2](https://github.com/jongbob1918/livox_mid360_gz_sim_ros2)). 직접 구현하면 Mid-360 격자(3601×595)가 스캔당 68.6 MB라 브리지가 불가능하다. 그래서 주력은 LIO-SAM 기본 가정(Velodyne 형식, ring, 10 Hz)과 맞고 렌더 비용이 작은 **VLP-16/XT16급 16채널 ±15°**로 둔다. 보고서에는 "16채널 회전형(단종 VLP-16 급)"으로 적는다.

| 항목 | 권장값 | 이유 |
|---|---|---|
| 센서 형식 | `gpu_lidar`, `<vertical>` samples 16, −0.261799 ~ +0.261799 rad | 3D는 vertical만 주면 됨. ring = 아래 빔부터 0–15 |
| 수평 | samples 1800, −π ~ π − 2π/1800 | 첫·끝 열 겹침 제거(현행 2D와 같은 규칙) |
| resolution | 1 (수평·수직 모두) | Fortress는 resolution≠1 보간을 하지 않음 ([Lidar.cc](https://github.com/gazebosim/gz-sensors/blob/ign-sensors6/src/Lidar.cc)) |
| range | min 0.10, max 30 m | 창고 60×40 m. SLAM 쪽 최소거리는 0.2 m로 따로 거름 |
| 잡음 | gaussian σ 0.01–0.03 m(현행 2D와 같은 0.03을 기본) | Fortress는 빔별 거리 가우시안만 지원 |
| 주기 | 10 Hz | LIO-SAM·RTAB-Map 예제 기대값 |
| 자기 가림 | `<visibility_mask>`로 자기 시각체 제외(현행 2D 방식) | 다른 로봇은 보이게 둠 |
| 장착 | 데크 위 마스트, 광학 중심 지면 +0.90 m | 아래 "장착 높이" 절 |
| 대상 | xacro 인자(예: `lidar3d:=true`), 기본값 false, 1대만 | 나머지 4대와 기존 시나리오 무영향 |

### Fortress gpu_lidar는 time 없는 순간 스캔과 ±inf 점을 낸다

Fortress(ign-sensors6) gpu_lidar의 점군 필드는 `xyz`·`intensity`(FLOAT32)와 `ring`(UINT16)뿐이고 **점별 time 필드가 없다**. `Update(_now)`는 `Render()`를 한 번 부르고 스탬프 하나를 붙인다. 따라서 스캔 전체가 한 시각, 한 자세에서 찍힌다. 범위 밖 빔은 ±inf 좌표로 남고, 하나라도 있으면 `is_dense=false`다 ([GpuLidarSensor.cc](https://github.com/gazebosim/gz-sensors/blob/ign-sensors6/src/GpuLidarSensor.cc)). 메모리 정렬 때문에 오프셋은 x=0, y=4, z=8, intensity=16, ring=24, point_step=32 B다. intensity는 시각체의 `laser_retro` 값이지 반사율 물리 모델이 아니다. ros_gz_bridge(humble)는 필드·오프셋·`is_dense`를 그대로 복사한다. 반면 다층 LaserScan은 "use the one in the middle"로 가운데 행만 넘긴다. 따라서 3D 데이터는 반드시 `<topic>/points`로 브리지한다 ([ros_gz convert/sensor_msgs.cpp](https://github.com/gazebosim/ros_gz/blob/humble/ros_gz_bridge/src/convert/sensor_msgs.cpp)). 점별 time을 넣어 달라는 gz-sensors 이슈 #507(2025-03)은 메인테이너 응답 없이 열려 있다. 댓글에 따르면 커스텀 GPU LiDAR를 만들려면 Sensors 시스템까지 새로 짜야 한다 ([gz-sensors #507](https://github.com/gazebosim/gz-sensors/issues/507)). 2026-05 gz-sensors main에 렌더링 없는 `CpuLidarSensor`가 병합됐지만 Jetty 이후용이고 필드도 같다 ([PR #593](https://github.com/gazebosim/gz-sensors/pull/593)).

정확도 문제도 하나 있다. gz-sim #2743은 Fortress·Harmonic의 gpu_lidar가 Classic보다 부정확하다고 보고했고, 원인과 수정은 없다 ([gz-sim #2743](https://github.com/gazebosim/gz-sim/issues/2743)). RTAB-Map의 Husky 데모는 렌더 엔진을 ogre1로 바꿔 우회한다. 그러나 우리는 헤드리스 EGL로 돌고, Fortress에서 EGL은 OGRE2 전용이라 이 우회를 쓸 수 없다 ([headless_rendering.md](https://github.com/gazebosim/gz-sim/blob/ign-gazebo6/tutorials/headless_rendering.md)). 대신 정답 기하와 gpu_lidar 점의 거리를 재서 센서 자체 오차를 수치로 남긴다(실행 계획 2단계).

이 특성에서 두 가지 설계가 나온다. 첫째, 프로젝트 자체 전처리 노드(rclcpp 컴포넌트, intra-process)를 둔다. 이 노드는 비유한 점을 지우고, ring을 유지하고, dense 비조직 점군으로 다시 내보내고, frame_id를 프로젝트 규약(`<prefix>lidar3d_link`)으로 맞춘다. 둘째, 점별 time은 넣지 않는 것이 기본이다. 순간 스캔에는 보정할 왜곡이 없기 때문이다. 공개 사례 multi-floor-lio-sam은 `time = col/width × 0.1 s`로 시간을 합성한다 ([multi-floor-lio-sam](https://github.com/wwwvvvvvvvv/multi-floor-lio-sam)). 이렇게 하면 LIO-SAM이 각 점을 IMU 회전 ω·t만큼 돌려 없는 왜곡을 "보정"한다. 명세 최대 1.5 rad/s에서는 스캔 끝 점이 0.15 rad 돌아가고, 10 m 거리에서 약 1.5 m 횡오차가 생긴다(코드상 계산). 비교 실험에서는 이 방식을 쓰지 않는다. 같은 이유로 **Gazebo로는 "deskew가 정확도를 얼마나 올리는가"에 답할 수 없다**. 그 한계를 보고서에 적는다. 꼭 필요하면 1 kHz 정답 자세로 점을 옮겨 왜곡을 일부러 넣는 오프라인 소실험을 따로 둔다.

### 장착 높이는 화물 가림과 통로 축 구속이 함께 정한다

차체는 지면 +0.03~+0.33 m이고, 대형 화물(0.40 m)을 실으면 상단이 +0.73 m다(`config/robot_params.yaml`, `models/box_large/model.sdf`). 데크에 바로 달면(H≈0.40) 적재 상태에 따라 화물이 수평 시야를 크게 가려 SLAM 비교가 적재 여부에 따라 달라진다. 마스트를 데크 한쪽 끝에 두고 화물 먼 모서리까지 0.6 m라 하면, −15° 빔이 화물을 넘으려면 H ≥ 0.73 + 0.6·tan15° ≈ 0.89 m가 필요하다. 앞 절의 localizability 계산도 16채널에서는 0.85 m가 가장 좋았다. 0.45 m는 데크·차체 가림이, 1.20 m는 상자 위를 지나치는 것이 문제였다. 그래서 **0.90 m**를 권한다. 그 높이에서 바닥 사각 반경은 0.90/tan15° ≈ 3.4 m다. 랙 상단(2.5 m)은 수평 약 6.0 m 밖부터 보인다. 통로 안에서 0.30 m 옆의 랙 면은 0.82–0.98 m 띠만 보이는데, 마침 데크 위 상자(상단 약 0.9 m)가 이 높이에 걸린다. 근거리 저상 장애물은 3D가 못 보므로 2D LiDAR(+0.20 m)와 RGB-D가 계속 맡는다. 업계의 2D/3D 분업과 같은 구조다.

좁은 통로 때문에 최소거리를 반드시 고쳐야 한다. 기본값인 LIO-SAM `lidarMinRange: 1.0`과 FAST-LIO `blind: 2.0`은 통로 안 랙 면(횡 0.30 m) 점을 모두 버린다 ([LIO-SAM params.yaml](https://github.com/TixiaoShan/LIO-SAM/blob/ros2/config/params.yaml)). RTAB-Map Husky 데모의 `Icp/RangeMin 0.5`도 마찬가지다. SLAM 쪽 최소거리는 0.2 m로 두고 Gazebo `range.min`은 그보다 작은 0.10 m로 둔다. 마스트와 화물이 0.2 m 안에 들어오지 않게 배치한다.

### IMU는 3D 로봇만 200 Hz로 올리고, 정답 방위는 버리고 다시 만든다

Fortress IMU는 가속도·자이로 6채널에만 잡음을 붙인다. 방위(orientation)는 기본으로 켜져 있고 잡음이 없으며, 기준은 센서의 초기 자세다 ([ImuSensor.cc](https://github.com/gazebosim/gz-sensors/blob/ign-sensors6/src/ImuSensor.cc), [Imu.cc](https://github.com/gazebosim/gz-sim/blob/ign-gazebo6/src/systems/imu/Imu.cc)). SDF 1.9 IMU에는 방위 잡음 요소가 없고 `<enable_orientation>`으로 끄는 것만 가능하다 ([imu.sdf](https://github.com/gazebosim/sdformat/blob/sdf12/sdf/1.9/imu.sdf)). 우리 저장소도 같은 이유로 EKF에서 IMU 방위 융합을 막아 두었다(`config/ekf.yaml`, `amr_gazebo.xacro`). 그런데 LIO-SAM은 IMU 방위를 첫 키프레임 roll/pitch 초기화, 매 스캔 roll/pitch slerp(`imuRPYWeight 0.01`), 회전 초기 추정에 쓴다. 쿼터니언 노름이 0.1 미만이면 "please use a 9-axis IMU!"를 내고 종료하므로 방위를 그냥 끌 수도 없다 ([mapOptmization.cpp](https://github.com/TixiaoShan/LIO-SAM/blob/ros2/src/mapOptmization.cpp), [utility.hpp](https://github.com/TixiaoShan/LIO-SAM/blob/ros2/include/lio_sam/utility.hpp)). RTAB-Map도 IMU를 주면 방위로 초기 자세를 바꾸고 프레임 간 회전 추정값을 IMU 회전으로 대체한다 ([rtabmap Odometry.cpp](https://github.com/introlab/rtabmap/blob/0.23.7/corelib/src/Odometry.cpp)).

그래서 3D 로봇의 IMU 체인은 이렇게 짠다. `imu/data_raw`(잡음+바이어스)를 프로젝트의 imu_filter_node(바이어스 보정)에 넣고, 그 출력을 imu_filter_madgwick(`use_mag:=false`)에 넣는다. madgwick은 입력에서 각속도·가속도만 읽고 방위는 필터 상태로 채우므로 정답 방위가 새지 않는다 ([imu_filter_ros.cpp](https://github.com/CCNYRoboticsLab/imu_tools/blob/humble/imu_filter_madgwick/src/imu_filter_ros.cpp)). 정답 방위를 쓴 실행은 "상한(오라클) 행"으로만 따로 보고한다. LIO-SAM이 권하는 200 Hz 이상을 맞추려고 3D 로봇만 IMU를 200 Hz로 올린다. 같은 센서 품질을 유지하려면 이산 σ를 √2배로 키운다(자이로 2.83×10⁻⁴ rad/s, 가속도 0.024 m/s²). 프로젝트의 정적 바이어스(0.10 m/s², 0.01 rad/s)는 LIO-SAM 바이어스 사전 σ=10⁻³보다 훨씬 크다. 그러니 바이어스를 보정한 `imu/data`를 넣고, `imuGravity`는 SDF 중력에 맞춰 9.8로 둔다. 모든 메시지가 같은 시뮬레이션 시계로 찍히므로 LiDAR–IMU 시간 오프셋은 0이다. `/clock` 브리지는 하나만 두고 IMU·점군 브리지가 겹치지 않게 한다. multi-floor-lio-sam은 브리지 중복으로 IMU가 두 번 들어와 `dt <= 0`과 GTSAM 예외를 겪었다 ([multi-floor-lio-sam README](https://github.com/wwwvvvvvvvv/multi-floor-lio-sam)).

### 3D 라이다는 1대에만 달고, 구독이 있을 때만 렌더한다

Fortress Sensors 시스템은 월드의 모든 렌더링 센서를 **렌더 스레드 하나**에서 처리한다. 렌더가 진행 중이면 물리 루프의 PostUpdate가 기다린다. 구독자가 있는 센서만 렌더한다 ([Sensors.cc](https://github.com/gazebosim/gz-sim/blob/ign-gazebo6/src/systems/sensors/Sensors.cc)). ros_gz #368은 Humble+Fortress에서 브리지를 켜면 RTF가 40–60%, 끄면 약 90%라고 보고했다. 다만 댓글에서 gz 기여자가 확인한 대로 브리지를 끈 상태는 센서가 아예 렌더하지 않는 상태다. 비용은 브리지가 아니라 "구독이 생기면 렌더와 전송이 일어나는 것"에서 온다 ([ros_gz #368](https://github.com/gazebosim/ros_gz/issues/368)). LaserScan 메시지도 구독자가 있을 때만 만든다 ([gz-sensors PR #448](https://github.com/gazebosim/gz-sensors/pull/448)). Ogre2 GpuRays 규칙으로 계산하면 16×1800 센서 하나는 큐브맵 4면(512²) 렌더와 광선 28,800개를 쓰고, 점군은 스캔당 0.92 MB, 10 Hz에서 9.2 MB/s다 ([Ogre2GpuRays.cc](https://github.com/gazebosim/gz-rendering/blob/ign-rendering6/ogre2/src/Ogre2GpuRays.cc)). 5대 모두 달면 브리지 데이터만 46 MB/s가 늘어난다. 이미 RTF가 0.2–0.3이므로 5대 전부 3D는 플릿 실험을 방해할 공산이 크다(추정, 실측 필요).

그래서 3D LiDAR는 SLAM 비교 대상 1대에만 달고, 브리지는 `lazy: true`로 둔다 ([ros_gz_bridge README](https://github.com/gazebosim/ros_gz/blob/humble/ros_gz_bridge/README.md)). 정확도 평가는 그 로봇 1대(와 동적 장애물)만 띄운 실행을 rosbag으로 녹화해 오프라인으로 재생한다. 부하는 3D 센서 N=0/1/5 × 브리지 켬/끔 × 10/5 Hz를 각 5분씩 돌려 gz `/stats` RTF, `ros2 topic hz`, nvidia-smi(공유 GPU), loadavg를 함께 기록한다. RTF가 1보다 작으면 SLAM 노드는 시뮬레이션 1초당 3–5배의 계산 시간을 얻는다. 그러니 "실시간 처리 가능" 여부는 bag을 `--rate 1.0`으로 재생해 따로 잰다.

### 정답 궤적은 이미 있고, 정답 지도는 생성 스크립트의 시각체에서 뽑는다

로봇 모델은 이미 OdometryPublisher로 `/<robot>/ground_truth/odom`(odom_frame=world, 3차원, 50 Hz)을 낸다. 이것은 스폰 위치와 무관한 월드 절대 자세이고 evo가 bag에서 바로 읽는다(`amr_gazebo.xacro`). 50 Hz보다 촘촘한 정답이 필요할 때만 PosePublisher를 더한다 ([PosePublisher.hh](https://github.com/gazebosim/gz-sim/blob/ign-gazebo6/src/systems/pose_publisher/PosePublisher.hh)). 지도 정답은 월드 생성 스크립트에서 바로 만든다. 월드가 박스 프리미티브로만 만들어지기 때문이다. 주의할 점은 랙이다. 충돌체는 2.0×1.0×2.5 m 통짜 박스지만, 시각체는 하단 0.5 m 데크, 0.08 m 기둥 4개, 선반 판 2장으로 열려 있다(`models/rack/model.sdf`). gpu_lidar는 렌더링 기반이라 **시각체**를 보므로 정답도 시각체로 만든다. `gen_warehouse_world.py`에 박스 목록을 OBJ/PLY로 내보내는 함수를 더하고, Open3D로 1 cm 간격 표면 표본을 뽑아 3D 정답 점군을 만든다. 동적 행위자와 런타임에 생성되는 화물은 뺀다. 2D 정답은 0.20 m 단면을 **0.01 m** 해상도로 래스터화한다. 지금처럼 추정 지도와 같은 0.05 m로 만들면 셀 양자화(±2.5 cm)가 재려는 ADNN(≈2.6 cm)과 같은 크기가 되기 때문이다. 다만 추정 지도의 셀 중심 규약 때문에 완전한 지도에서도 생기는 바닥값(+0.7~+3.4 cm)은 GT 해상도를 높여도 그대로 남는다(`slam.md` §3).

## 주력은 RTAB-Map, 명세 기준선은 LIO-SAM이고 나머지는 이유를 남기고 뺀다

### Humble에서 실제로 깔리는 3D SLAM은 생각보다 많고 판이 둘이다

"Humble apt로 설치되는 3D LIO/SLAM은 GLIM·MOLA·RKO-LIO뿐"이라는 초기 조사 결론은 틀렸다. 공식 apt에는 **RTAB-Map**(rtabmap_ros, BSD, `rtabmap_examples/lidar3d.launch.py`)과 **Cartographer**(2.0.9002, Apache-2.0, 3D 설정 포함)도 있다 ([rosdistro humble](https://github.com/ros/rosdistro/blob/master/humble/distribution.yaml)). 판 번호에도 함정이 있다. rosdistro master에 적힌 판은 ros2-testing에만 있고, 2026-10-06 현재 일반 apt(main)는 한 단계 낮다. main은 `ros-humble-rtabmap*` 0.23.7, `ros-humble-mola` 3.2.0, `ros-humble-mola-lidar-odometry` 3.2.0, `ros-humble-rko-lio` 0.4.0, `ros-humble-mp2p-icp` 2.14.0이다. testing은 각각 0.23.13, 3.3.1, 3.3.0, 0.4.2, 3.0.1이다 ([apt main Packages.gz](http://packages.ros.org/ros2/ubuntu/dists/jammy/main/binary-amd64/Packages.gz), [ros2-testing Packages.gz](http://packages.ros.org/ros2-testing/ubuntu/dists/jammy/main/binary-amd64/Packages.gz)). 예를 들어 RKO-LIO 0.4.0에는 launch 인자 `use_sim_time`이 없다(0.4.1에서 추가). 토픽 `rko_lio/frame`도 0.4.1에서 `rko_lio/deskewed_scan`으로 바뀌었다 ([rko_lio CHANGELOG](https://github.com/PRBonn/rko_lio/blob/master/CHANGELOG.rst)). 그러니 착수할 때 쓸 판을 정해 기록하고, 문서는 그 판의 태그로 읽어야 한다. LIO-SAM, FAST-LIO, KISS-ICP, Kinematic-ICP, GLIM, lidar_localization_ros2는 rosdistro에 없다.

| 후보 | Humble 설치 | 라이선스 | Gazebo 입력 적합성 | 휠 사전값·평면 제약 | 위치추정·Nav2 경로 | 결정 |
|---|---|---|---|---|---|---|
| RTAB-Map | 공식 apt(main 0.23.7 / testing 0.23.13) | BSD | 비유한 점 자동 제거, ring 무시. `deskewing:=false` 필수 | `guess_frame_id`로 휠-EKF, 저복잡도 축은 guess 값 사용, `Reg/Force3DoF` | 위치추정 모드, map→odom 또는 `localization_pose`, 높이 대역 2D 격자 | **주력** |
| LIO-SAM (ros2) | 소스 빌드(GTSAM 4.1 PPA 또는 rosdep 4.2.0) | BSD-3 | `sensor: velodyne`이면 그대로 들어감. time 없으면 deskew 끔 | 휠 없음. z·roll·pitch 자르기만 | 위치추정 모드 없음. PCD 저장 | **명세 기준선(매핑 비교 행)** |
| RKO-LIO | 공식 apt(main 0.4.0) | MIT | `deskew=false`면 헤더 시각으로 채움. 6축만 씀 | 없음 | 오도메트리만 | 예비: 휠 없는 최신 LIO 행 |
| MOLA-LO | 공식 apt(main 3.2.0) | GPLv3 + CLA | 2D·3D 파이프라인 동봉 | 휠 속도 융합, `enforce_planar_motion` | 위치추정 전용 모드, map→odom, 격자는 파이프라인 직접 작성 | 예비: 같은 알고리즘 2D/3D 쌍 |
| Kinematic-ICP | 소스 빌드 | MIT | 시각이 모두 같으면 0 나눗셈 → time 필드 없이 | 휠 필수, 유니사이클 평면 | 오도메트리만 | 예비: 창고 근거가 있는 휠 결합 오도메트리 |
| GLIM | 저자 PPA | MIT | 지원 | 휠 모듈 없음 | 사전지도 위치추정 없음 | 제외 |
| FAST-LIO2 (ROS2) | 소스 빌드, livox_ros_driver2 필수 | GPL-2.0 | time 없으면 가짜 시각 합성, inf 검사 없음 | 없음 | 없음 | 제외(문헌 비교에만 인용) |
| Cartographer 3D | 공식 apt 2.0.9002 | Apache-2.0 | IMU 필수, inf 검사 없음 | 외부 odom 위 map→odom 가능 | 3D 서브맵 투영에 높이 대역 없음 | 제외 |
| lidar_localization_ros2 | 소스 빌드 | BSD-2 | 지원 | 외부 odom 필요 | Nav2 모드 map→odom | 참고만 |
| KISS-ICP / KISS-SLAM | 소스 / pip | MIT | time 없으면 deskew 끔 | 없음 | 없음 / ROS 노드 없음 | 제외(RKO-LIO로 대체) |

### RTAB-Map을 주력으로 고른 다섯 가지 이유와 두 가지 약점

첫째, 설치와 의존성이 가장 깔끔하다. 공식 apt 바이너리는 `ros-humble-libpointmatcher`와 `ros-humble-gtsam`(4.2.0)에 의존한다. rosdep의 GTSAM과 같은 판이라 PPA GTSAM 4.3과 충돌할 일이 없다 ([apt main Packages.gz](http://packages.ros.org/ros2/ubuntu/dists/jammy/main/binary-amd64/Packages.gz)). 둘째, Gazebo 점군을 그대로 받는다. `icp_odometry`는 `ring`을 읽지 않고, 변환 단계(`laserScanFromPointCloud`, `filterNaNs=true`)에서 NaN과 ±inf를 버린다 ([icp_odometry.cpp](https://github.com/introlab/rtabmap_ros/blob/0.23.13-humble/rtabmap_odom/src/nodelets/icp_odometry.cpp), [rtabmap util3d.cpp](https://github.com/introlab/rtabmap/blob/0.23.7/corelib/src/util3d.cpp)). 셋째, 기존 휠-EKF를 정합 안에 넣을 수 있다. `guess_frame_id`로 받은 휠 TF를 초기값으로 쓴다. 구조 복잡도가 `Icp/PointToPlaneMinComplexity`(0.02)보다 낮으면 `Icp/PointToPlaneLowComplexityStrategy=1`이 작동한다. 원문 설명은 "for a corridor-like environment, the resulting transform will be limited in y and yaw, x will taken from the guess"다 ([Parameters.h](https://github.com/introlab/rtabmap/blob/0.23.7/corelib/include/rtabmap/core/Parameters.h), [RegistrationIcp.cpp](https://github.com/introlab/rtabmap/blob/0.23.13-humble/corelib/src/RegistrationIcp.cpp)). 퇴화 완화 현장 비교(T-FR 2025)가 "튜닝이 어려우면 등식 제약"이라고 권한 계열과 같다 ([Tuna et al., T-FR 2025](https://doi.org/10.1109/TFR.2025.3576053)). 넷째, Nav2 통합 경로가 한 패키지에 다 있다. 높이 대역 2D 격자, 위치추정 모드, `initialpose` 구독, Humble+Fortress+3D LiDAR+Nav2 데모(Husky)와 TurtleBot3 Nav2 데모가 있다 ([husky_sim_scan3d_demo.launch.py](https://github.com/introlab/rtabmap_ros/blob/humble-devel/rtabmap_demos/launch/husky/husky_sim_scan3d_demo.launch.py)). 다섯째, 같은 `icp_odometry`가 LaserScan도 받는다. `guess_frame_id`와 `Reg/Force3DoF`를 그대로 둔 채 2D와 3D 입력만 바꿀 수 있어 차원 효과를 깔끔하게 떼어 낼 수 있다. testing 판 0.23.13에는 bag 재생 헬퍼를 포함한 노드 단위 테스트와 Humble CI도 새로 들어왔다 ([rtabmap_ros 0.23.13-humble](https://github.com/introlab/rtabmap_ros/tree/0.23.13-humble)).

약점도 분명하다. 첫째, 2023–2026년에 RTAB-Map의 3D LiDAR(ICP) 모드를 FAST-LIO2·LIO-SAM·MOLA와 같은 데이터에서 비교한 독립 벤치마크는 이번 탐색에서 찾지 못했다. IILABS 3D와 MapsHD는 RTAB-Map을 넣지 않았다. 1차 정량 근거는 저자 논문(JFR 2019)뿐이다. KITTI에서 단일 코어 S2M ICP 오도메트리는 ATE 0.7–17.2 m, 평균 82 ms였다. 평균 병진 오차 0.56–2.52%는 LOAM(0.57–1.43%)보다 대체로 나빴다 ([Labbé & Michaud, JFR 2019](https://doi.org/10.1002/rob.21831)). RTAB-Map은 정확도 1위가 아니라 통합·운용성 1위 후보이고, 정확도 근거는 우리 자체 벤치가 1차 근거가 된다. 둘째, LiDAR만 쓰면 전역 재위치추정이 없다. 루프 검출의 BoW가 카메라에 의존하기 때문이다. LiDAR는 현재 추정 근처의 proximity 검출만 하므로, 초기 자세가 거울 자세면 그대로 굳는다(4장에서 다룸). 덧붙여 전략 1은 등식 제약 계열이라 성능이 휠 품질에 달려 있다. 우리 시뮬 휠은 오라클 수준이라 이 경로가 지나치게 좋게 나올 수 있다(5장).

다음 값은 출발값이다. 예제와 데모, 우리 기하에서 나왔고 튜닝 bag에서 확정한다.

| 노드·항목 | 출발값 | 근거·주의 |
|---|---|---|
| `icp_odometry` 입력 | `scan_cloud`=전처리된 3D 점군, `frame_id=amr_0k/base_link`, `odom_frame_id=icp_odom`, `guess_frame_id=amr_0k/odom` | guess 지정 시 `icp_odom→odom` 보정 TF만 내고 odom→base_link는 EKF가 유지 ([test_odometry_ros.cpp](https://github.com/introlab/rtabmap_ros/blob/0.23.13-humble/rtabmap_odom/test/test_odometry_ros.cpp), [#1417 메인테이너 답](https://github.com/introlab/rtabmap_ros/issues/1417)) |
| deskew | `deskewing:=false` | 예제 기본 true면 time 필드가 없어 "Failed to deskew input cloud, aborting odometry update!"로 프레임을 버림 ([MsgConversion.cpp](https://github.com/introlab/rtabmap_ros/blob/0.23.13-humble/rtabmap_conversions/src/MsgConversion.cpp)) |
| IMU | `imu_topic` 비움(기본). 쓸 때는 madgwick 출력만 | IMU 방위를 직접 씀 |
| ICP | `Icp/Strategy 1`, `Icp/PointToPlane true`, `Icp/VoxelSize 0.1`, `Icp/MaxCorrespondenceDistance 1.0`, `Icp/PointToPlaneK 20`, `Icp/PointToPlaneGroundNormalsUp 0.9`, `Icp/RangeMin 0.2` | 예제 실내 권장 0.1–0.3. 통로 0.30 m 때문에 최소거리 0.2 ([lidar3d.launch.py](https://github.com/introlab/rtabmap_ros/blob/humble-devel/rtabmap_examples/launch/lidar3d.launch.py)) |
| 평면·차동구동 | `Reg/Force3DoF true`, `Odom/Holonomic false` | Husky 데모의 "2D flat floor" 설정 ([husky_slam3d.launch.py](https://github.com/introlab/rtabmap_ros/blob/humble-devel/rtabmap_demos/launch/husky/husky_slam3d.launch.py)) |
| 퇴화 축 | `Icp/PointToPlaneLowComplexityStrategy 1`, `icpStructuralComplexity` 기록 | 퇴화 플래그 지표로 씀 |
| 오루프 방지 | `RGBD/OptimizeMaxError 0.3–1.0`, 루프 `Icp/CorrespondenceRatio ≥0.3`, `RGBD/ProximityPathMaxNeighbors 1`, `RGBD/MaxOdomCacheSize 10` | Husky 데모가 반복 패턴 때문에 0.3 사용 |
| 격자 | `RGBD/CreateOccupancyGrid true`, `Grid/Sensor 0`, `Grid/3D false`, `Grid/CellSize 0.05`, `Grid/RangeMax 20`, `Grid/RangeMin 0.2`, `Grid/NormalsSegmentation false`, `Grid/MaxGroundHeight 0.05`, `Grid/MaxObstacleHeight 0.40`(비교용) / `2.0`(운용용), `Grid/RayTracing true`, `Grid/Footprint{Length,Width,Height} 0.60/0.40/0.30` | 예제는 격자를 끄고, 기본 `Grid/RangeMax`가 5 m임 ([Parameters.h](https://github.com/introlab/rtabmap/blob/0.23.7/corelib/include/rtabmap/core/Parameters.h)) |
| 위치추정 모드 | `Mem/IncrementalMemory False`, `Mem/InitWMWithAllNodes True`, `Rtabmap/DetectionRate` 1→2–5 Hz 검토 | 기본 1 Hz |

기동 확인은 `ros2 launch rtabmap_examples lidar3d.launch.py lidar_topic:=<points> frame_id:=<base_link> deskewing:=false use_sim_time:=true`로 시작한다. 예제가 인자로 노출하지 않는 값이 많으므로 확인 뒤에는 예제를 복사해 프로젝트 launch로 만든다.

### LIO-SAM은 별도 컨테이너에서, 방위를 바꿔서 돌린다

명세가 "LIO-SAM 등"을 들었으므로 LIO-SAM은 반드시 비교표에 넣는다. ros2 브랜치(HEAD 08af3f3, 2025-02-14)는 README에 "humble on Ubuntu 22.04" 시험을 적었다. GTSAM은 README 기준 `ppa:borglab/gtsam-release-4.1`이고, package.xml의 rosdep 키는 `ros-humble-gtsam` 4.2.0으로 풀린다 ([LIO-SAM ros2](https://github.com/TixiaoShan/LIO-SAM/tree/ros2), [PR #534](https://github.com/TixiaoShan/LIO-SAM/pull/534)). 어느 쪽이 런타임에 문제가 없는지 공식 언급은 없다. Humble+Fortress에서 실제로 돈 공개 사례(multi-floor-lio-sam)는 GTSAM 4.1.1을 썼다. 그래서 LIO-SAM은 RTAB-Map과 다른 이미지(`wf-final` 기반)에 GTSAM 4.1로 빌드하고 녹화된 bag에만 돌린다. 이렇게 하면 우리 colcon 작업공간의 "경고 0" 제약과도 분리된다.

초기 노트의 "is_dense=false면 종료하므로 NaN 제거 노드가 필요하다"는 틀렸다. ros2 브랜치는 dense 검사 바로 앞에서 `pcl::removeNaNFromPointCloud`를 부르고, PCL 1.12.1의 이 함수는 NaN·±inf를 지운 뒤 `is_dense=true`로 바꾼다 ([imageProjection.cpp](https://github.com/TixiaoShan/LIO-SAM/blob/ros2/src/imageProjection.cpp), [PCL filter.hpp](https://github.com/PointCloudLibrary/pcl/blob/pcl-1.12.1/filters/include/pcl/filters/impl/filter.hpp)). 실제로 걸리는 곳은 다른 데다. `ring` 형식 일치(`sensor: velodyne`), IMU 쿼터니언 노름 검사, 최소거리다. 처리 비용은 FAST-LIO2 논문 표에서 매핑 스캔당 95–325 ms(i7-8550U)였고, LIO-SAM 원 논문에서는 41.9–100.5 ms(i7-10710U)였다 ([FAST-LIO2, arXiv 2107.06829](https://arxiv.org/abs/2107.06829), [LIO-SAM, arXiv 2007.00258](https://arxiv.org/abs/2007.00258)). 다만 기본 `mappingProcessInterval 0.15` s와 `numberOfCores 4`가 있어 스캔당 시간 × 10 Hz로 계산하면 과대 추정이다. 강건성에는 경고가 따른다. 퇴화 데이터셋 GEODE의 Offroad-α 7개 시퀀스에서 LIO-SAM은 5개가 붕괴했다. 저자들은 LIO-SAM의 퇴화 검출이 퇴화 방향을 제대로 식별하지 못한다고 관찰했다 ([GEODE, arXiv 2409.04961](https://arxiv.org/abs/2409.04961)). Gazebo Ignition에서 녹화한 bag이 "queue is full"로 메시지를 버리며 드리프트한다는 이슈 #497도 답 없이 열려 있다 ([LIO-SAM #497](https://github.com/TixiaoShan/LIO-SAM/issues/497)). 그러니 재생 속도를 낮추고 수신·처리 스캔 수를 센다.

| 항목 | 기본값 | 우리 값 | 이유 |
|---|---|---|---|
| `sensor` | ouster | velodyne | Gazebo ring은 uint16(ouster 형식은 uint8) |
| `N_SCAN` / `Horizon_SCAN` | 64 / 512 | 16 / 1800 | 가상 센서와 일치 |
| `lidarMinRange` / `lidarMaxRange` | 1.0 / 1000 | 0.2 / 30 | 통로 랙 면 0.30 m |
| `extrinsicRot` | [−1,0,0; 0,1,0; 0,0,−1] | 단위행렬 | 시뮬은 축이 정렬됨 |
| `extrinsicRPY` | [0,1,0; −1,0,0; 0,0,1] | 단위행렬 | 두 기본값이 서로 다름에 주의 |
| `extrinsicTrans` | 저자 장비 | URDF의 lidar3d_link–imu_link 차이 | 마스트 높이만큼 큼 |
| `imuTopic` | — | madgwick 출력 | 정답 방위 누출 차단 |
| IMU 주기 | (권장 ≥200 Hz) | 200 Hz, σ √2배 | 3D 로봇만 |
| `imuGravity` | 9.80511 | 9.8 | SDF 중력 |
| IMU 잡음 파라미터 | acc 3.99e-3, gyr 1.56e-3 | 기본 유지 | 우리 연속 밀도(1.7e-3, 2.0e-5)보다 커서 IMU를 덜 믿는 안전한 쪽 |
| `z_tollerance`, `rotation_tollerance` | 1000 | 기본 유지. 평면 변형은 별도 행 | 켜면 평면 사전을 주는 튜닝 손잡이 |
| 루프 폐쇄 | on | on/off 둘 다 | 오도메트리 단계와 SLAM 단계 분리 |
| `use_sim_time` | launch 인자 없음 | `--ros-args -p use_sim_time:=true` | |
| 퇴화 플래그 | — | `lio_sam/mapping/odometry_incremental`의 `pose.covariance[0]` 기록 | 코드 수정 없이 퇴화 빈도 측정 |

multi-floor-lio-sam은 "Waiting for IMU data"를 imageProjection의 `cloudQueue.size() <= 2`를 5로 바꿔 해결했다고 적었다. 하지만 이 저장소에는 LICENSE 파일이 없어 코드를 복사할 권리가 없다. 아이디어만 참고한다. time 필드를 넣지 않으면 IMU 요구 범위가 스캔 시작 시각 한 점으로 줄어 이 문제도 덜 생긴다. LIO-SAM은 위치추정 모드가 없다. 그래서 역할은 **매핑 궤적과 지도 품질 비교**로 한정한다. 지도는 `lio_sam/save_map` 서비스로 PCD를 저장한다. 그다음 octomap_server의 `projected_map`(`occupancy_min_z/max_z`를 반드시 지정, 기본 ±100 m)이나 자체 스크립트로 같은 높이 대역의 2D 격자를 만든다 ([octomap_server.cpp](https://github.com/OctoMap/octomap_mapping/blob/ros2/octomap_server/src/octomap_server.cpp)).

### 예비 후보는 같은 bag에 붙이는 비교 행으로만 쓴다

**RKO-LIO**는 "휠 없는 최신 LIO" 행으로 가장 싸다. 공식 apt, MIT, RA-L 2026 게재다. 문서는 "A 6-axis IMU is enough; we don't use orientation"이라 정답 방위가 샐 경로가 없다. `deskew=false`면 모든 점의 시각을 헤더 시각으로 채워 time 필드 없이 돈다 ([rko_lio data.rst](https://github.com/PRBonn/rko_lio/blob/master/docs/pages/data.rst), [RA-L DOI](https://doi.org/10.1109/LRA.2026.3685966)). 기본 `voxel_size`는 1.0 m이고 문서가 실내에서는 줄이라고 권하므로 줄여서 기록한다 ([config.rst](https://github.com/PRBonn/rko_lio/blob/master/docs/pages/config.rst)). TF를 항상 방송하니 `odom_frame` 이름을 바꾸고 `invert_odom_tf`를 써서 EKF의 odom과 충돌하지 않게 한다. 논문 성능은 장면마다 엇갈린다. Oxford Spires Blenheim에서는 ATE 0.20 m로 FAST-LIO2(0.94 m)보다 좋았지만, Bodleian에서는 0.86 m로 FAST-LIO2(0.26 m)보다 나빴다 ([arXiv 2509.06593](https://arxiv.org/abs/2509.06593)).

**MOLA**는 같은 프레임워크에 2D 파이프라인(`lidar2d.yaml`)과 3D 파이프라인이 함께 있다. 휠 속도 융합과 `enforce_planar_motion`도 갖춰 RTAB-Map과 다른 구현으로 차원 효과를 교차 확인할 수 있다 ([mola_lidar_odometry](https://github.com/MOLAorg/mola_lidar_odometry), [IJRR 2025](https://doi.org/10.1177/02783649251316881)). apt main 3.2.0에도 위치추정 전용 모드(`start_mapping_enabled`), 초기 지도 로드, REP-105 map→odom(`publish_localization_following_rep105`, 기본 True)이 실제로 있다 ([mola_lo_ros_node.rst 3.2.0](https://github.com/MOLAorg/mola_lidar_odometry/blob/3.2.0/docs/mola_lo_ros_node.rst)). 제약은 셋이다. REP-105 모드는 Simple 상태추정기와만 맞고, MOLA-LO는 GPLv3 + CLA다. 또 `mm2grid`는 이미 있는 격자 레이어만 내보내므로 `FilterVoxelSlice`로 격자 레이어를 만드는 파이프라인 YAML을 직접 짜야 한다 ([mm2grid](https://github.com/MOLAorg/mp2p_icp/blob/2.14.0/docs/source/app_mm2grid.rst), [FilterVoxelSlice.cpp](https://github.com/MOLAorg/mp2p_icp/blob/2.14.0/mp2p_icp_core/mp2p_icp_filters/src/FilterVoxelSlice.cpp)). 기본 사전값 가중 `icp_prior_weight 1.0`은 코드 주석 그대로 "breaks ties rather than constraining anything"이라, 통로 퇴화를 막으려면 키워야 한다 ([acceptance.yaml](https://github.com/MOLAorg/mola_lidar_odometry/tree/develop/pipelines/blocks)).

**Kinematic-ICP**는 창고 실측 근거가 있는 유일한 휠 결합 오도메트리다. 휠 TF가 필수이고 `use_2d_lidar:=true`로 2D 입력도 받으며, bag을 직접 읽는 `offline_node`가 있다 ([kinematic-icp](https://github.com/PRBonn/kinematic-icp)). 시뮬 데이터에는 함정이 있다. 점 시각이 모두 같으면 `(t−min)/(max−min)`이 0 나눗셈이 되어 죽는다(열린 PR #32). 그러니 time 필드를 아예 넣지 않는다 ([PR #32](https://github.com/PRBonn/kinematic-icp/pull/32)). 코드의 `fixed_regularization`은 논문 식의 1/β에 해당하는 가중치로 보인다. 논문의 β=0.01을 재현하려면 약 100을 넣어야 할 것으로 읽힌다(코드 읽기 기반 추론, 미검증). IILABS 실험실 벤치마크에서는 ATE 0.147–0.676 m로 LIORF(0.025–0.035 m)보다 나빴다. 실기 휠 미끄러짐 시퀀스가 최악이었다 ([IILABS 3D](https://jorgedfr.github.io/3d_lidar_slam_benchmark_at_iilab/)). 예비 후보의 역할은 이처럼 휠 품질이 결과를 지배하는 정도를 드러내는 것이다.

### 뺀 것과 그 이유를 기록한다

**GLIM**은 GPU 가속 인자 그래프로 문헌상 퇴화에 강하다. 하지만 근거 실험은 폭 40 m 공간에서 사거리를 15 m로 잘라 복도 가운데에서 몇 초 동안 완전 퇴화를 만든 조건이었고 루프 폐쇄는 꺼져 있었다. 그 조건에서 LIO-SAM 1.473 m, FAST-LIO2 1.294 m, GLIM 0.099 m였다 ([GLIM, arXiv 2407.10344](https://arxiv.org/abs/2407.10344)). 양쪽 랙이 보이면서 진행축만 약한 0.60 m 통로와는 다르다. 결정적인 문제는 운용이다. 저자는 사전지도 위치추정이 GLIM에 없고 유료 폐쇄 소스 GLIL에 있다고 답했다 ([glim #84](https://github.com/koide3/glim/issues/84)). 저자 PPA는 `libgtsam-notbb-dev 4.3.0`을 `/usr/local`에 깔아 `ros-humble-gtsam 4.2.0`과 한 컨테이너에 공존하게 만든다 ([PPA Packages](https://koide3.github.io/ppa/ubuntu2204/Packages)). 휠 모듈도 없어 저자가 "odom 인자 가중을 키우거나 VGICP에 가중 래퍼를 씌우라"고 답한 수준이다 ([glim #191](https://github.com/koide3/glim/issues/191)). CUDA 판은 SASS sm_75–90과 PTX만 담고 있어, RTX 5090에서는 세 판 모두 PTX JIT로 돌 것으로 예상된다([NVIDIA Blackwell 호환 가이드](https://docs.nvidia.com/cuda/blackwell-compatibility-guide/index.html)). 실측은 하지 않았다.

**FAST-LIO2**는 독립 비교가 가장 풍부한 LIO다. DLIO 논문 표에서 Newer College 5개 시퀀스 모두 LIO-SAM보다 정확했고, 평균 시간은 42.86 대 179.33 ms였다. GEODE Offroad-α 7개도 모두 살아남았다 ([DLIO, arXiv 2203.03749](https://arxiv.org/abs/2203.03749), [GEODE](https://arxiv.org/abs/2409.04961)). 그러나 ROS2 브랜치는 GPL-2.0이다(package.xml은 BSD로 잘못 표기). livox_ros_driver2가 빌드 필수이고 `-std=c++14`를 강제한다. 무엇보다 time 필드가 없으면 방위각으로 점 시각을 합성하는데, 시계방향 회전을 가정한다 ([preprocess.cpp](https://github.com/hku-mars/FAST_LIO/blob/ROS2/src/preprocess.cpp)). Gazebo 점은 반시계로 채워지므로 링마다 0 ms 다음에 99.7 ms, 그 뒤 199.3 ms에서 99.8 ms로 거꾸로 줄어드는 엉뚱한 시각이 만들어진다(코드 추적). isfinite 검사도 없어 inf 점이 지도에 들어간다. 모든 점에 1 µs 상수 time을 넣고 inf를 거르면 쓸 수는 있다. 그래도 같은 "휠 없는 LIO" 자리를 RKO-LIO가 apt로 채우므로 통합 비용이 이득보다 크다.

**Cartographer 3D**는 기능상 불가가 아니라 유지보수 대비 이득이 없어서 뺀다. 업스트림 README가 2024-01에 "Cartographer is no longer actively maintained"라고 공지했고, ROS 포크도 "limited capacity"다 ([cartographer README](https://github.com/cartographer-project/cartographer/blob/master/README.rst)). 3D는 IMU가 필수다 ([algo_walkthrough](https://github.com/ros2/cartographer_ros/blob/ros2/docs/source/algo_walkthrough.rst)). 온라인 격자 노드가 쓰는 3D 서브맵 텍스처는 높이 대역 선택이 없는 투영이라 Nav2 격자로 바로 쓰기 어렵다 ([submap_3d.cc](https://github.com/cartographer-project/cartographer/blob/master/cartographer/mapping/3d/submap_3d.cc)). 같은 기능을 RTAB-Map이 한 패키지로 준다. **lidar_localization_ros2**는 Nav2 모드 map→odom과 PCD→점유격자 생성기를 갖췄다. 그러나 저자 스스로 "Jazzy is the primary target"이라 하고, Humble CI는 빌드만 하며, 문서는 "not production-grade global relocalization"이라고 한정한다 ([v1_status.md](https://github.com/rsasaki0109/lidar_localization_ros2/blob/main/docs/v1_status.md)). 그래서 참고 자료로만 둔다.

라이선스는 다음 원칙으로 관리한다. apt 바이너리는 작업공간 밖에 있으므로 그 경고가 우리 빌드 로그에 섞이지 않는다(명세의 "경고 0"이 우리 패키지만 대상인지는 확인 필요). LIO-SAM은 별도 이미지에서 빌드한다. GPL 코드(MOLA-LO, FAST-LIO 계열)는 저장소에 복사하지 않는다. evo(GPL-3.0)는 평가 도구로만 쓴다. MapEval은 README가 MIT라고 하지만 저장소에 LICENSE 파일이 없다. 그러니 지표는 논문 정의대로 Open3D로 직접 구현한다 ([Cloud_Map_Evaluation](https://github.com/JokerJohn/Cloud_Map_Evaluation)).

## 3D 지도는 두 높이 대역의 2D 격자와 하나의 map→odom으로 Nav2에 들어간다

### 비교용 격자와 운용용 격자를 따로 뽑는다

RTAB-Map은 노드마다 로컬 격자를 만들고 최적화된 그래프 자세로 전역 격자에 합친다. 3D 점군은 `Grid/MaxGroundHeight`~`Grid/MaxObstacleHeight` 대역으로 바닥과 장애물을 나눈 뒤 xy로 투영한다(`Grid/3D=false`). 결과는 latched `nav_msgs/OccupancyGrid` `/map`으로 나간다 ([MapsManager.cpp](https://github.com/introlab/rtabmap_ros/blob/0.23.13-humble/rtabmap_util/src/MapsManager.cpp)). 이 대역을 두 번 다르게 뽑는다. **비교용 격자(0.05–0.40 m)**는 로봇 높이 0.30 m에 여유를 둔 충돌 관련 대역이다. 랙 하단 블록, 기둥, 벽이 들어가 2D 지도와 같은 높이대 구조를 담는다. 정답도 같은 대역의 시각체 footprint 합집합으로 새로 래스터화해 비교한다. **운용용 격자(0.05–2.0 m)**에는 랙 선반 돌출부까지 들어간다. Nav2 안전 관점에서는 이쪽이 맞지만, 0.20 m 단면 정답과는 체계적으로 달라진다. 두 대역의 재현율 차이를 내면 "2D가 못 보는 구조물의 비율"이 수치로 나온다. 내보내기는 `ros2 run nav2_map_server map_saver_cli -f <이름> --ros-args -r map:=/rtabmap/map`으로 하고, YAML origin을 스폰 정답 자세로 맞춘 뒤 `map_quality.py`에 넣는다.

동적 장애물 잔상은 2D와 3D에 같은 조건으로 다뤄야 한다. RTAB-Map 전역 격자는 hit 0.7 / miss 0.4로 갱신된다. 한 번만 보고 다시 지나지 않은 셀은 잔상으로 남는다 ([Parameters.h](https://github.com/introlab/rtabmap/blob/0.23.7/corelib/include/rtabmap/core/Parameters.h)). 그러니 매핑 주행에서는 동적 장애물을 끄거나 같은 구역을 두 번 이상 지나는 경로를 쓴다. slam_toolbox 지도도 같은 bag으로 만든다. KISS-SLAM의 실험이 보여 주듯 "3D 지도를 높이 대역으로 잘라 기존 2D 위치추정기에 넣는 것" 자체는 이미 검증된 경로다 ([arXiv 2503.12660](https://arxiv.org/abs/2503.12660)).

### 위치추정은 AMCL 자리만 바꾸는 구성으로 비교한다

Nav2의 규약은 하나다. "The `map` to `odom` transform is provided by a positioning system … and `odom` to `base_link` by an odometry system"이고, LiDAR 기반 위치추정을 요구하지 않는다 ([Nav2 State Estimation](https://github.com/ros-navigation/docs.nav2.org/blob/rolling/docs/getting_started/navigation_concepts/state_estimation.md)). RTAB-Map은 이 규약에 세 가지 방식으로 붙는다. 구성 A에서는 `rtabmap`(위치추정 모드)이 로컬 EKF의 `/odometry/filtered`를 구독해 map→odom만 낸다. 메인테이너가 재현을 확인한 사용자 사례가 있다 ([#1387](https://github.com/introlab/rtabmap_ros/issues/1387)). 구성 B에서는 `icp_odometry`를 `guess_frame_id=odom`으로 끼워 `icp_odom→odom` 보정만 내고, 그 위에서 `rtabmap`이 `map→icp_odom`을 낸다. Husky·TurtleBot3 데모가 이 구조다. 구성 C에서는 `publish_tf:=false`로 두고 `/rtabmap/localization_pose`를 robot_localization 글로벌 EKF의 `pose0`로 넣는다. 메인테이너가 포럼에 올린 예제다(ROS 1 XML) ([RTAB-Map 공식 포럼](http://official-rtab-map-forum.206.s1.nabble.com/Filtering-rtabmap-localization-jumps-with-robot-localization-in-2D-td5931.html)).

우리 시스템에 맞는 배치는 단계마다 다르다. **매핑**은 구성 B로 한다(LiDAR로 다듬은 오도메트리 + 그래프 SLAM). **기동 확인**은 구성 A로 한다. 움직이는 부분이 가장 적어 map→odom이 맞게 나오는지 먼저 볼 수 있다. **2D와의 비교와 운용**은 구성 C로 한다. 현행 2D 스택은 AMCL 자세를 글로벌 EKF에 넣고 납치 감지 상태기가 그 잔차를 감시한다. 구성 C는 AMCL 자리에 RTAB-Map 자세만 꽂으므로 글로벌 EKF·상태기 코드를 그대로 쓰고, 2D와 3D의 차이가 "절대 자세 공급원" 하나로 좁혀진다. 대가는 공분산 튜닝이다. `localization_pose`는 위치추정이 일어날 때만 나오는 이벤트성 출력(기본 `Rtabmap/DetectionRate` 1 Hz)이다. 메인테이너도 점프 완화에 `RGBD/MaxOdomCacheSize`를, 빈도 부족에 DetectionRate 상향을 권하고 "you need appropriate covariance in your odometry topic"이라고 적었다 ([#719](https://github.com/introlab/rtabmap_ros/issues/719)). 어느 구성이든 **map→odom 발행자는 하나**여야 한다. 3D 로봇에서는 AMCL을 띄우지 않는다.

### Humble Nav2는 navigation_launch.py로 띄운다

초기 노트의 "`bringup_launch.py use_localization:=False`"는 Humble에서 동작하지 않는다. 이 인자는 Jazzy부터 있다. Humble의 `bringup_launch.py`는 `slam`이 거짓이면 `localization_launch.py`(map_server + amcl)를 그대로 include한다 ([navigation2 humble bringup_launch.py](https://github.com/ros-navigation/navigation2/blob/humble/nav2_bringup/launch/bringup_launch.py)). 3D 로봇은 `navigation_launch.py`만 띄운다. 이 파일은 controller·planner·behavior·bt_navigator 등만 올리고 map_server와 amcl은 없다 ([navigation_launch.py](https://github.com/ros-navigation/navigation2/blob/humble/nav2_bringup/launch/navigation_launch.py)). 지도는 RTAB-Map의 latched `/map`을 쓰거나, 저장한 PGM/YAML을 별도 nav2_map_server + lifecycle_manager로 띄운다. RTAB-Map의 TurtleBot3 데모와 Clearpath Humble Nav2 데모가 이 방식을 쓴다 ([turtlebot3_sim_scan_demo](https://github.com/introlab/rtabmap_ros/blob/humble-devel/rtabmap_demos/launch/turtlebot3/turtlebot3_sim_scan_demo.launch.py), [clearpath nav2.launch.py](https://github.com/clearpathrobotics/clearpath_nav2_demos/blob/humble/launch/nav2.launch.py)). 구성 B를 운용에 쓰면 TB3 데모처럼 local costmap의 `global_frame`을 `icp_odom`으로 바꿔야 한다. 프레임은 `frame_id=amr_0k/base_link`, `odom_frame_id=amr_0k/odom`, `map_frame_id=map`으로 둔다. 프로젝트가 전역 `/tf`와 프레임 접두어 방식을 쓰면 Husky 데모의 `/tf`→`tf` 리맵은 하지 않는다. 이 부분은 develop 브랜치 launch가 정리된 뒤 다시 확인해야 한다.

### 3D 점군은 STVL로 국소 코스트맵에 넣고, 2D 스캔은 근접 계층으로 남긴다

Nav2 튜닝 가이드는 `VoxelLayer`가 "Not suitable for 3D lidars due to sparseness"라고 하고, 3D LiDAR에는 Spatio-Temporal Voxel Layer를 권한다 ([Nav2 tuning guide](https://github.com/ros-navigation/docs.nav2.org/blob/rolling/docs/configuration_and_development/tuning_guide.md)). STVL 튜토리얼은 3D 센서 처리 자원을 "up to 2x" 줄인다고 하고, `voxel_decay`, `voxel_size 0.05`, `min/max_obstacle_height` 예시를 준다 ([STVL 튜토리얼](https://github.com/ros-navigation/docs.nav2.org/blob/rolling/docs/tutorials/general_tutorials/navigation2_with_stvl.md)). Humble apt판은 2.3.4(LGPL-2.1)다. 2026-10-02의 data race 수정 커밋은 apt 빌드보다 늦어 바이너리에 들어 있지 않다 ([STVL humble](https://github.com/SteveMacenski/spatio_temporal_voxel_layer/tree/humble)). 3D 로봇의 local costmap에 STVL을 더하되, 2D 스캔은 근거리 장애물 소스로 남긴다. 업계 분업(2D = 근접·안전, 3D = 오버행·원거리)과 1:1로 맞는 구성이다. 다만 랙 하단 0.5 m 블록은 0.20 m 평면에도 잡히므로 "오버행" 효과를 보이려면 별도 장애물이 필요하다. 0.3–1.0 m 높이로 튀어나온 적재물이나 들어 올린 지게차 포크가 그 예다. 3D 센서에 2D 알고리즘을 쓰는 기준선(`pointcloud_to_laserscan`으로 높이 띠를 LaserScan으로 만들어 현행 AMCL에 넣는 것)도 만들 수 있다. 이때 `min_height` 기본값이 가장 작은 양수 double이고 `scan_time` 기본값이 1/30 s라는 함정이 있다. 높이 띠, `scan_time 0.1`, `target_frame`을 반드시 명시한다 ([pointcloud_to_laserscan humble](https://github.com/ros-perception/pointcloud_to_laserscan/tree/humble)).

### 전역 재위치추정은 3D로 풀리지 않으니 기존 복구 상태기를 유지한다

LiDAR 단독 RTAB-Map은 카메라 BoW 없이는 현재 추정 근처의 proximity만 검출한다. 저자 논문도 "a camera is always required even if lidar SLAM is done … relying only on proximity detection"이라고 적는다 ([Labbé & Michaud, JFR 2019](https://doi.org/10.1002/rob.21831)). 그러니 초기 자세가 맞으면 오루프가 막히고, 거울 자세로 틀리면 그대로 굳는다. `RGBD/StartAtOrigin`이 기본 false라 위치추정 모드는 지난 세션의 마지막 자세에서 다시 시작하기도 한다. 해결은 바깥에서 한다. `rtabmap`은 `initialpose`를 구독하므로, 기존 납치 감지·전역 시드·수렴 게이트 상태기가 AMCL에 주던 초기 자세를 같은 토픽으로 RTAB-Map에 주면 된다 ([CoreWrapper.cpp](https://github.com/introlab/rtabmap_ros/blob/0.23.13-humble/rtabmap_slam/src/CoreWrapper.cpp)). 대칭을 지도 쪽에서 깨는 선택지도 있다. RGB-D 카메라를 함께 구독해 `RGBD/MarkerDetection=true`로 ArUco 마커를 랜드마크로 쓰고, `Marker/Priors`에 월드 생성 스크립트에서 뽑은 81개 마커의 정답 자세를 넣는다. 이렇게 하면 거울 대칭을 깨는 전역 제약과 월드 프레임 고정을 함께 얻는다 ([Parameters.h](https://github.com/introlab/rtabmap/blob/0.23.7/corelib/include/rtabmap/core/Parameters.h)). 하지만 이것은 "2D 대 3D" 비교에 마커라는 교란 변수를 넣는다. 그래서 마커를 끈 조건을 주 결과로, 켠 조건을 별도 행으로 보고한다. 공개 실험이 없어 해소 여부 자체도 추론이다.

## 공정성은 휠 사전값, IMU 방위, 정렬 클래스에서 갈린다

### 한 번 기록하고 여러 번 재생한다

비교의 기본은 open-loop 재생이다. 3D 로봇 1대의 Gazebo 주행을 시나리오마다 한 번 녹화한다. bag에는 2D 스캔, 3D 점군(전처리 전), 원시 IMU, 휠 엔코더 odom, EKF odom, `ground_truth/odom`, TF를 함께 넣는다. 그 bag을 slam_toolbox→AMCL 2D 파이프라인, RTAB-Map 2D/3D, LIO-SAM 등에 재생하면 입력·궤적·동적 장애물이 완전히 같아진다. 주행 경로를 `route_driver`가 정답 되먹임으로 정하므로 경로가 추정기와 독립이고, 그래서 이 방식이 가능하다. Nav2 폐루프 결과(목표 도달, 충돌)는 재생할 수 없으므로 별도 폐루프 실험으로 확인한다.

### 시뮬레이션의 세 가지 오라클을 걷어낸다

첫째는 **IMU 방위**다. 앞서 본 대로 madgwick 재추정으로 막고, 정답 방위 실행은 오라클 행으로 따로 둔다. 둘째는 **휠**이다. 우리 휠 모델은 물리 슬립을 쓰지 않고 소프트웨어 노드가 오차를 넣는다. `slip_noise_stddev 0.01`(기준 굴림 0.01 m), `slip_distance_coeff 0`, `wheel_param_rel_stddev 0`이다(`wheel_odometry.yaml`). 이번 자체 계산으로는 차체 전진 오차가 0.71 mm·√(D/m)이다. 1 m에 0.07%이고, 4 m 통로 전체에서 1.4 mm다. Kinematic-ICP 표의 실기 휠 RPE 0.89–4.86%보다 1–2자릿수 작다 ([arXiv 2410.10277](https://arxiv.org/abs/2410.10277)). 그래서 RTAB-Map 전략 1, Kinematic-ICP, MOLA+휠처럼 휠을 쓰는 방법은 통로에서 사실상 휠 정확도를 물려받는다. 퇴화 완화 현장 비교의 결론도 "잡음 있는 사전값에서는 등식 제약 외 대부분이 발산하고, 결론은 사전값 품질에 달려 있다"였다 ([Tuna et al., T-FR 2025](https://doi.org/10.1109/TFR.2025.3576053)). 따라서 반드시 **열화 휠** 수준을 같이 돌린다. 이미 있는 손잡이로 좌우 바퀴 반경을 1% 어긋나게 하고 `slip_distance_coeff`·`wheel_param_rel_stddev`를 0보다 크게 줘, 휠 RPE가 실기 범위(약 1–3%)에 들게 맞춘다. 셋째는 **순간 스캔**이다. 2D와 3D 모두 왜곡이 없으니 둘 사이의 형평은 유지된다. 다만 deskew의 가치는 평가할 수 없다는 한계를 적는다.

### 요인 설계로 차원 효과를 떼어 낸다

2D 쪽(slam_toolbox, AMCL, 자체 정합)은 이미 휠-EKF odom을 쓴다. 그러므로 주 비교에서는 3D에도 같은 휠-EKF 사전값을 준다. 동시에 사전값 유무를 요인으로 두어야 "차원 효과"와 "사전값 효과"가 섞이지 않는다.

| 요인 | 수준 | 구현 |
|---|---|---|
| A 센서 | 2D(+0.20 m, 720빔, 25 m) / 3D(+0.90 m, 16×1800, 30 m) | 같은 bag, RTAB-Map `icp_odometry`에 scan 또는 cloud |
| B 휠 사전값 | 휠-EKF guess 있음 / 없음 | `guess_frame_id` 유무 |
| C 자유도 | SE(2) / SE(3) | `Reg/Force3DoF` |
| D 휠 품질 | 현행(사실상 오라클) / 열화(RPE 1–3%) | `wheel_odometry.yaml` 손잡이 |
| E 통로 끝 | 열림 / 대기 로봇·작업자로 막음 | 시나리오 S6 |

전부 교차하면 너무 크다. 핵심은 세 묶음이다. ① A×B(D 현행, E 열림)는 "휠을 같이 주면 2D와 3D가 거의 같다"는 예상을 확인한다. ② 같은 A×B를 D 열화로 다시 돌린다. ③ A×B를 E 막힘에서 돌린다. ②와 ③이 3D의 이점이 드러날 조건이다. 여기에 고정 행 네 개를 더한다. 현행 2D 스택(slam_toolbox → AMCL + 이중 EKF + 정합), 명세 행 LIO-SAM(휠 없음, madgwick 방위, 루프 on/off), 예비 행(RKO-LIO, MOLA 2D/3D, Kinematic-ICP 2D/3D), 그리고 정답 IMU 방위를 쓴 오라클 행이다. 사거리 효과를 통제하려면 3D 사거리를 2D와 같은 25 m로 자른 행도 하나 둔다. 상류 오도메트리가 61–75% 좋아져도 SLAM 뒤에는 차이가 훨씬 작아졌다는 2026년 결과가 있다 ([Merrick & Nandikolla, Sensors 2026](https://doi.org/10.3390/s26175468)). 그러니 오도메트리 단계(루프 off)와 SLAM 단계(루프 on)를 둘 다 보고한다.

시나리오는 일곱 가지다. S1 정상(동적 장애물 정지), S2 동적(사람 0.3–1.2 m/s·지게차·셔틀, 씨드가 다른 bag 3개 이상), S3 좁은 통로 왕복(통로 안 정지 포함, 통로 축/횡 오차 분해), S4 점대칭 별칭(스폰 자리와 거울 자세 근처에서 전역 위치추정 시작), S5 납치(기존 8개 목표 + 거울 자세 근처 목표), S6 통로 끝 막힘, S7 돌출 장애물(Nav2 폐루프)이다. 매핑에는 기존 322 m 매핑 경로를 쓴다.

### 지표는 정렬 없는 SE(2) 오차와 같은 격자 공간의 지도 지표를 주 결과로 둔다

| 대상 | 주 지표 | 보조 지표 | 주의 |
|---|---|---|---|
| 매핑 궤적 | 정렬 없는 SE(2) ATE(우리 map = world) | `--align_origin`, SE(2) Umeyama, RPE 1/2/5/10/20 m, 10 s 창 RTE | 3D의 z·roll·pitch는 진단용으로 따로 |
| 2D 지도 | `map_quality.py` ADNN₀/ADNN, Chamfer, IoU·정밀도·재현율(τ=1셀) | 기둥 쌍 거리, 외벽 직선성·간격 | GT 0.01 m, 셀 중심 바닥값 명시 |
| 3D 지도 | F-score @5·10 cm, AC/COM | Chamfer, 유령 점 비율(SA/DA/AA) | GT 관측영역 마스크는 별도 조작으로 명시 |
| 위치추정 | 명세 구간(정지/직선/회전) RMSE·max·p95 vs 3/5/8 cm | GT 지도 위 vs SLAM 지도 위 차이, 도크 반복정밀도, 완료율 | 정렬 없음. 구간 판정은 `segments.py` 임계 |
| 강건성 | 실패율(Wilson 95% CI), CRT | 별칭률, 납치 복구 성공률·시간, 퇴화 플래그와 기하 localizability 일치율 | 실패 실행을 평균에서 빼지 않음 |
| 자원 | 스캔당 지연 p50/p95, CPU%(코어 정규화), 최대 RSS | 지도 크기, RTF(N=0/1/5) | `--rate 1.0`, 코어 고정 |

정렬은 결과를 숨길 수 있다. GNSS로 전역 좌표가 걸린 RTK-SLAM 데이터셋에서 SE(3) 정렬은 절대 위치 오차를 최대 76% 과소평가했다 ([Zhang et al., arXiv 2604.07151](https://arxiv.org/abs/2604.07151)). 우리 계약은 map = world이므로 정렬 없는 값이 주 결과다. 정렬 클래스는 추정기의 비관측 자유도에 맞춰야 한다 ([Zhang & Scaramuzza, IROS 2018](https://doi.org/10.1109/IROS.2018.8593941)). 2D 추정에는 z·roll·pitch가 없으므로 정렬을 하더라도 SE(2)로 한다. evo v1.37.1에는 SE(2) CLI 옵션이 없고, `--project_to_plane xy`는 3D 정렬 **뒤에** 적용된다. 그러니 3D 궤적에 `-a`를 주면 정렬 자유도가 2D와 달라진다. Python API `umeyama_alignment(est[:, :2].T, ref[:, :2].T)`는 차원 일반이라 그대로 SE(2) 정렬이 된다. 직선 구간만 따로 정렬하면 랭크 1로 실패하므로 구간별 정렬은 하지 않는다. 함정이 두 가지 더 있다. `-s` 단독은 축척만 보정한다(Sim(3)은 `-a -s`). `--sync_method interpolation`은 배포판이 아닌 2026-09 master에만 있다 ([evo](https://github.com/MichaelGrupp/evo)). rpg_trajectory_evaluation은 `eval_cfg.yaml`이 없으면 기본이 sim3다 ([rpg_trajectory_evaluation](https://github.com/uzh-rpg/rpg_trajectory_evaluation)). ATE는 초반 회전 오차를 과대 반영하고 원위치 복귀 시 우연히 낮아질 수 있으므로, 오도메트리 드리프트에는 10 s 창 RTE를 함께 낸다 ([Potokar & Kaess, IROS 2025](https://arxiv.org/abs/2507.16000)).

지도 쪽에서 2D와 3D를 같은 저울에 올리는 공통 척도는 "같은 2D 격자 공간으로 투영한 뒤의 지표"뿐이다. 3D 전용 지표로는 Oxford Spires 관례대로 F-score를 5·10 cm 임계에서 낸다. 그 논문의 필터링은 재구성 점 가운데 GT 영역 밖에 떨어진 것만 지우는 한쪽 방향 조작이다 ([Oxford Spires, arXiv 2411.10546](https://arxiv.org/abs/2411.10546)). 우리가 GT를 "주행 궤적에서 보이는 면"으로 깎는다면 다른 조작이므로 따로 이름 붙여 보고한다. 지도 엔트로피(MME)는 같은 도구·같은 반경·같은 센서 밀도의 3D 후보끼리만 비교한다. MapEval 코드의 기본 반경은 0.1 m이고 원 정의(Droeschel)는 0.5 m라 값이 바로 비교되지 않는다 ([MapEval](https://doi.org/10.1109/LRA.2025.3548441)). 동적 장애물이 남긴 유령 점은 DynamicMap 벤치마크의 SA·DA·AA=√(SA·DA)로 잰다. 시뮬레이션은 행위자의 정답 자세를 알므로 점마다 동적 라벨을 붙일 수 있다 ([DynamicMap Benchmark, arXiv 2307.07260](https://arxiv.org/abs/2307.07260)). 위치추정은 GT 지도 위와 SLAM 지도 위에서 각각 돌린다. 그 차이가 "지도 품질이 위치추정에 주는 몫"이고, 현재 도크 쪽 4.23 cm 문제가 바로 이 몫이다.

### 재생을 통제하고 표본을 충분히 모은다

재생에는 숨은 누락 경로가 많다. rosbag2 Humble의 `--clock`은 기본으로 발행하지 않는다. 값 없이 플래그만 주면 40 Hz다 ([rosbag2 humble play.py](https://github.com/ros2/rosbag2/blob/humble/ros2bag/ros2bag/verb/play.py)). TF 조회 양자화를 줄이려면 `--clock 200` 정도로 준다. 녹화 bag에 `/clock`이 들어 있으면 재생에서 빼야 시간원이 둘로 갈리지 않는다. mcap을 쓰려면 `rosbag2_storage_mcap`을 따로 설치해야 하고, 기본은 sqlite3다. slam_toolbox 동기 모드의 "모든 스캔 처리"는 tf2 MessageFilter(큐 `scan_queue_size`, humble 기본 1)와 최소 이동거리·시간 간격 필터를 통과한 스캔에 한한다 ([slam_toolbox humble](https://github.com/SteveMacenski/slam_toolbox/tree/humble)). 정확도 실행은 `--rate 0.5` 이하로 하고, 노드별 수신·필터 통과·처리 스캔 수를 세어 **누락 0**을 확인한다. 자원 실행은 `--rate 1.0`에 `--cpuset-cpus`로 코어를 고정하고, 구성 순서를 ABAB로 섞어 공유 서버 부하의 시간 추세를 상쇄한다. 공유 서버라 DVFS를 끌 수 없다는 점은 한계로 적는다. 자원 비교에서 DVFS를 끄고 의존 라이브러리 판을 고정한 선례가 있다 ([Bujanca et al., IROS 2021](https://arxiv.org/abs/2109.13160)).

표본은 시나리오마다 **씨드가 다른 bag 3–5개 × bag당 재생 3–5회 = 구성당 10–25개**로 잡는다. 재생 간 분산(알고리즘·ROS 비결정성)과 bag 간 분산(장면 다양성)은 따로 보고한다. 비결정적 로봇 시험 사례 연구에서는 반복 실행보다 입력 다양성이 결함을 찾는 데 더 효율적이었다 ([Robert et al., LAAS](https://hal.science/hal-02444350/document)). 표본 수를 계산하면, 짝지은 차이의 표준편차가 0.5 cm일 때 0.5 cm 차이를 α=0.05, 검정력 0.8로 잡으려면 n≈(1.96+0.84)²≈7.8이고 t 보정으로 약 10이다. 예비 5회로 σ_d를 재고 다시 계산한다 ([Colas et al., arXiv 1806.08295](https://arxiv.org/abs/1806.08295)). 검정은 같은 bag끼리 짝지은 Wilcoxon 부호순위(정규성이 확인되면 짝 t)로 하고, 부트스트랩 95% CI와 효과크기를 함께 낸다. 지표와 시나리오가 여럿이므로 Holm 보정을 한다. 실패율과 별칭률은 Wilson 95% CI로 낸다. 같은 bag·같은 구성을 세 번 돌려 결과가 같은지 먼저 보고, 다르면 그 산포를 "재현 잡음"이자 차이 판정 문턱으로 쓴다.

시뮬레이션 결론은 실데이터 한 번으로 교차 확인하면 설득력이 커진다. IILABS 3D 데이터셋에는 3D LiDAR 4종과 함께 Hokuyo UST-10LX 2D LiDAR(LaserScan 40 Hz), Xsens IMU 400 Hz, 휠 오도메트리, OptiTrack 정답이 같은 bag에 들어 있다 ([IILABS 데이터셋](https://jorgedfr.github.io/3d_lidar_slam_benchmark_at_iilab/content/dataset/)). 2D 사거리가 10 m라 우리 25 m와 다르므로, 비교할 때 사거리를 맞춰 자른다. 보고는 ISO 18646-2:2024의 형식을 따른다. 이 표준은 이동 로봇 항법 성능을 자세 정확도·반복정밀도, 좁은 통로, 지도 정확도로 재고 시험 조건을 보고서에 밝히라고 한다 ([ISO 18646-2:2024 미리보기](https://cdn.standards.iteh.ai/samples/82643/d7c0eb63bf4e4a6984bd529e4957706b/ISO-18646-2-2024.pdf)). 정답 정확도가 시험 대상보다 10배 좋아야 한다는 규칙은 T&E 4Log 논문이 ISO/IEC 18305를 인용한 것으로만 확인된다. 위치추정 시스템용 규칙이므로 지도 GT 해상도에 쓰는 것은 유추라고 밝힌다 ([T&E 4Log, arXiv 2107.10597](https://arxiv.org/abs/2107.10597)).

## 포인트 클라우드 처리와 최적화는 여섯 개의 절제 실험으로 배운다

명세의 "3D 포인트 클라우드 처리 및 최적화 학습"은 따로 공부할 주제가 아니라 위 파이프라인의 손잡이를 하나씩 돌려 보는 절제 실험으로 채우는 것이 가장 실속 있다. 각 실험은 S1·S3 bag 하나와 RTAB-Map 3D 구성에서 한 손잡이만 바꾸고, ATE·지도 F-score·스캔당 지연을 같이 기록한다.

| 주제 | 돌릴 손잡이 | 측정 | 배울 내용과 근거 |
|---|---|---|---|
| 입력 위생과 데이터 구조 | 비유한 점 제거 on/off, 조직/비조직 점군, intra-process | 수신 점 수, 지연, 실패 여부 | PointCloud2 메모리 배치(point_step 32 B, 정렬 패딩), Gazebo ±inf 처리. FAST-LIO처럼 isfinite 검사가 없는 코드에서 무엇이 깨지는지 |
| 다운샘플링 | `Icp/VoxelSize` 0.05/0.1/0.2/0.3 | ATE, 지연, 대응 비율 | 정확도-속도 곡선. RTAB-Map 예제는 실내 0.1–0.3 권장 ([lidar3d.launch.py](https://github.com/introlab/rtabmap_ros/blob/humble-devel/rtabmap_examples/launch/lidar3d.launch.py)), RKO-LIO 기본 1.0 m는 실내에서 줄이라고 문서화 |
| 정합 오차 척도 | 점-대-점 vs 점-대-평면, `Icp/PointToPlaneK` | ATE, 통로 축 오차 | 정형 환경에서는 평면 특징과 plane-to-plane 잔차가 가장 좋았다는 절제 결과 ([Potokar & Kaess](https://arxiv.org/abs/2507.16000)) |
| 지면·높이 대역 분리 | passthrough vs 법선 분할, `Grid/MaxObstacleHeight` 0.40/1.0/2.0 | 2D 격자 IoU·재현율 | 대역 선택이 지도 품질과 Nav2 안전을 어떻게 바꾸는지 |
| 동적 물체 제거 | 레이트레이싱 on/off, DUFOMap 후처리 on/off | 유령 점 SA/DA/AA, 격자 정밀도 | DUFOMap은 오프라인 AA 98.34/96.12/92.70/90.94%, 온라인 질의에서는 95.31/89.50/85.43/73.98%로 떨어지고 반실내 DA는 54.76% ([DUFOMap, arXiv 2403.01449](https://arxiv.org/abs/2403.01449)). 신규 프로젝트에는 ERASOR2가 권장됨 ([ERASOR](https://github.com/LimHyungTae/ERASOR)). 제거는 2D 슬라이스 이전에 해야 함 |
| 그래프 최적화와 루프 검증 | `RGBD/OptimizeMaxError` 0.3/1.0/3.0, 루프 `Icp/CorrespondenceRatio` | 정답 기준 오루프 수, 지도 이중벽 | 반복 랙에서는 "루프를 많이 찾기"보다 "오루프를 넣지 않기"가 중요. 반복 환경 자기유사성 가지치기 ([MapClosures, arXiv 2501.07399](https://arxiv.org/abs/2501.07399)), 검증 전 오루프 후보 거르기로 인자 정밀도 0.542→0.717 ([QCSA, arXiv 2512.09447](https://arxiv.org/abs/2512.09447)) |

이 여섯 실험에 퇴화 감지 비교 하나를 덧붙이면 최적화 쪽 학습이 완성된다. RTAB-Map의 `icpStructuralComplexity`, LIO-SAM의 퇴화 플래그, 그리고 1장의 정답 기하 localizability를 경로 위에서 겹쳐 그린다. 그러면 "알고리즘이 퇴화라고 한 구간"과 "기하상 퇴화인 구간"의 일치율이 나온다. GICP 계열의 Hessian 기반 감지는 공분산 정규화 때문에 병진 퇴화를 가린다는 2026년 프리프린트가 있어, 감지 결과를 그대로 믿지 않는 이유도 함께 배울 수 있다 ([LF-GICP, arXiv 2608.19522](https://arxiv.org/abs/2608.19522)).

## 열한 단계로 나눈 실행 계획과 단계별 통과 기준

순서는 "센서와 정답을 먼저 믿을 수 있게 만들고, 오프라인 재생으로 비교한 뒤, 마지막에 Nav2 폐루프로 확인"이다. 최소 성공 범위(MVP)는 0–6단계다. RTAB-Map 3D·RTAB-Map 2D·slam_toolbox·LIO-SAM 네 행을 S1·S3·S6 시나리오에서 A×B 요인(휠 현행)으로 비교한다. 나머지는 확장으로 둔다.

| 단계 | 할 일 | 산출물 | 통과 기준 |
|---|---|---|---|
| 0. 결정 동결과 환경 | `wf-final` 기반 이미지에 `ros-humble-rtabmap-ros`, `imu-tools`, STVL, pointcloud_to_laserscan, octomap_server, `rosbag2_storage_mcap` 설치. main/testing 중 쓸 판 결정. evo 판 고정. LIO-SAM은 별도 이미지에 GTSAM 4.1로 빌드 | 판 기록표(패키지·판·커밋), 이미지 2개 | 두 이미지에서 노드 기동. 우리 작업공간 colcon 경고 0 유지 |
| 1. 센서 모델 | xacro `lidar3d:=true`(기본 false), 마스트 0.90 m, gpu_lidar SDF(표 참고), 3D 로봇 IMU 200 Hz(σ √2), `/points`만 lazy 브리지, 전처리 컴포넌트(비유한 점 제거, frame_id) | 센서 xacro, 브리지 YAML, 전처리 노드 | `/points` 10 Hz, 필드 xyz/intensity/ring 확인, 전처리 후 `is_dense=true`. 다른 4대와 기존 시나리오 무변화 |
| 2. 부하와 센서 오차 측정 | N=0/1/5 × 브리지 켬/끔 × 10/5 Hz 각 5분. gpu_lidar 점과 정답 면 거리 측정(#2743 정량화) | RTF·토픽 주기·GPU·loadavg 표, 센서 오차 분포 | 1대 장착 시 RTF 하락 폭 기록. 센서 거리 오차 중앙값·p95 기록, 잡음 σ보다 큰 계통 편차는 원인 확인 |
| 3. 정답 생성기 | `gen_warehouse_world.py`에 시각체 메시 내보내기, 3D 표면 점군(1 cm), 2D 단면(0.20 m) 0.01 m 래스터, 비교용 대역 래스터, 거울 자세 정합 점수 지도(선택) | GT 파일 세트 | 먼저 0.05 m GT로 현행 slam_toolbox 지도의 기존 값(ADNN₀ 2.59 cm 등)을 재현해 생성기를 검증한 뒤 0.01 m로 전환 |
| 4. 데이터 기록 | S1–S6 시나리오, 씨드별 bag. 2D 스캔·3D 점군·원시 IMU·휠·EKF odom·GT odom·TF | bag 세트와 메타데이터(씨드·RTF) | 토픽별 메시지 수와 주기 확인, `/clock` 처리 규칙 고정 |
| 5. 매핑 행 기동 | RTAB-Map 3D(구성 B), RTAB-Map 2D(같은 설정, scan 입력), slam_toolbox(동기, `scan_queue_size` 상향), LIO-SAM(madgwick, 루프 on/off) | 행별 궤적, 2D 격자(두 대역), 3D 점 지도 | "Failed to deskew" 없음, 처리 스캔 누락 0, TF 트리 유효, 지도 원점 = 월드 |
| 6. 지도·궤적 평가 | 정렬 없는 SE(2) ATE·RPE·RTE, `map_quality.py`, 3D F-score 스크립트, 유령 점 비율 | 결과 CSV와 표 | 행·지표마다 구성당 표본 ≥10, 재현 잡음 산포 기록 |
| 7. 위치추정 비교 | RTAB-Map 위치추정 모드(구성 A로 확인 후 C), GT 지도 위·SLAM 지도 위 각각, 명세 구간 RMSE·max·p95, S4 별칭·S5 납치 | 위치추정 결과표 | map→odom 발행자 하나, 3/5/8 cm 판정을 RMSE와 max 둘 다로 기록 |
| 8. Nav2 폐루프 | `navigation_launch.py` + RTAB-Map 지도, 3D 로봇 local costmap에 STVL, S7 돌출 장애물, 기존 시나리오 하네스 일부 | 목표 도달·충돌·경로 이탈 비교 | 2D 대비 회귀 없음, 돌출 장애물 회피 차이 기록 |
| 9. 확장(선택) | D 열화 휠, E 막힘 전체 요인, RKO-LIO·MOLA·Kinematic-ICP 행, 정답 방위 오라클 행, 25 m 사거리 절단 행, Marker/Priors 조건, IILABS 실데이터 교차 확인, 6장 절제 실험 | 확장 결과표 | 사전 등록한 비교만 검정, Holm 보정 |
| 10. 보고 | 조건표, 정렬 명시, 분포(중앙값[IQR]·평균±SD·RMSE·max·p95), 실패 분리, 판 기록, 스크립트·씨드 공개 | 보너스 2 보고서·슬라이드 | 과목 형식(슬라이드 요구 항목, 평이한 국문)으로 압축 |

## 위험과 대안, 그리고 정직하게 주장할 수 있는 범위

| 위험 | 징후 | 대응 | 안 될 때의 대안 |
|---|---|---|---|
| 3D 렌더로 RTF 추가 하락 | 2단계 측정에서 RTF 급락 | 1대만, lazy 브리지, 녹화·재생 평가 | 수평 1024열 또는 5 Hz, 녹화 실행은 그 로봇만 띄움 |
| gpu_lidar 계통 오차(#2743) | 정답 면 거리 편차 > 잡음 σ | 센서 오차를 따로 보고, 양쪽 지도에 같은 영향임을 명시 | ogre1 우회는 EGL에서 불가하므로 수치로 감수 |
| RTAB-Map deskew 기본값 | 오도메트리 갱신 중단 로그 | `deskewing:=false` | — |
| 정답 IMU 방위 누출 | 3D 행만 roll/pitch·yaw 오차가 비현실적으로 작음 | madgwick 재추정, 오라클 행 분리 | IMU 없이 `Reg/Force3DoF` 경로 |
| 시뮬 휠 오라클 | 휠 쓰는 행이 통로에서 mm급 | 열화 휠 요인 추가 | WheelSlip 물리 슬립을 일부 시나리오에만(팀 합의 필요) |
| LIO-SAM 빌드·런타임 | GTSAM 링크 오류, "Waiting for IMU data", queue full | 별도 이미지 GTSAM 4.1, 느린 재생, 큐 여유 | evalio(ROS 없이 LIO-SAM을 같은 bag에 돌림, MIT) ([evalio](https://github.com/contagon/evalio)) |
| apt 판 차이 | 문서 인자가 없음 | 판 동결, 그 판 태그로 문서 확인 | ros2-testing 또는 소스 빌드 |
| Humble Nav2 인자 부재 | `use_localization` 무시 | `navigation_launch.py` 직접 구성 | — |
| 거울 자세 별칭 | 위치추정 모드가 거울에 굳음 | 기존 복구 상태기 → `initialpose` | `Marker/Priors` 조건(교란 변수로 별도 보고) |
| 동적 잔상 | 격자 정밀도 하락, 유령 점 | 매핑 시 동적 끔 또는 2회 통과, 2D에도 같은 조건 | DUFOMap 후처리 |
| 위치추정 공분산(구성 C) | 글로벌 EKF가 자세를 거부하거나 튐 | DetectionRate 상향, 공분산 조정, MaxOdomCacheSize | 구성 A로 비교 |
| 통계 검정력 부족 | CI가 0을 넘나듦 | 예비 5회로 σ_d 추정 후 n 재계산 | "차이 없음(검정력 x)"으로 정직하게 보고 |
| 라이선스 | GPL 코드 혼입 | 복사 금지, 도구로만 사용 | MIT/BSD 행으로 대체 |

| 주장 | 가능 여부 | 조건 |
|---|---|---|
| 3D LiDAR를 시뮬레이션에 추가하고 3D SLAM(RTAB-Map, LIO-SAM)을 적용했다 | 가능 | 판·설정·센서 사양 공개 |
| 같은 입력에서 2D 대비 지도 품질·위치 정확도를 비교했다 | 가능 | 정렬 클래스, 휠 사전값, IMU 방위 처리를 조건표에 명시 |
| 3D는 스캔 평면 위 구조(선반, 돌출물)를 지도·코스트맵에 담는다 | 가능 | 대역별 재현율과 S7 결과로 |
| 막힌 통로 끝에서 3D가 통로 축을 더 잘 구속한다 | 측정 후 가능 | S6 결과와 localizability 계산을 함께 |
| 정상 조건에서 2D와 3D 정확도가 비슷하다 | 측정 후 가능 | 짝지은 검정과 CI. "같다"가 아니라 "차이가 x 이하로 검출되지 않음" |
| 3D가 일반적으로 더 정확하다 | 불가 | 시뮬 휠·순간 스캔·단일 월드 조건 |
| 3D가 점대칭 별칭을 해결한다 | 불가(가설) | 비대칭 3D 구조가 있는 영역에서만, S4 측정 후 범위를 한정해 |
| deskew가 정확도를 높인다 | 불가 | Gazebo 순간 스캔 |
| 실기에서 실시간으로 돈다 | 불가 | `--rate 1.0` 재생 결과는 "이 호스트에서"로 한정 |
| 5대 동시 3D 운용이 된다 | 측정 전 불가 | 2단계 N=5 측정 후에만 |
| 3D로 안전 기능을 대체한다 | 불가 | 제3자 인증 광학 3D LiDAR 최고 PL b, 시뮬은 안전 검증 대상 아님 |

## 결론

이번 조사로 보너스 2의 성격이 바뀌었다. "LIO-SAM을 돌려 3D가 낫다는 그림을 얻는 과제"가 아니라 "시뮬레이션이 만드는 세 가지 오라클(정답 IMU 방위, 이상적인 휠, 왜곡 없는 순간 스캔)을 걷어낸 상태에서 차원 효과를 다른 변수와 떼어 내는 통제 실험"이다. 그래서 주력 도구의 기준도 정확도 순위가 아니라 통합성과 통제 가능성이 된다. RTAB-Map이 주력인 이유는 FAST-LIO2나 GLIM보다 정확해서가 아니다. 같은 알고리즘에 2D와 3D를 바꿔 넣고, 휠 사전값과 자유도를 손잡이 하나로 켜고 끄며, 그 결과를 기존 글로벌 EKF와 Nav2에 AMCL 자리만 바꿔 꽂을 수 있기 때문이다. LIO-SAM은 명세 문구를 충족하는 비교 행이지, 운용 경로가 아니다.

예상되는 결론이 "정상 조건에서는 2D와 3D가 비슷하다"라도 실패가 아니다. 같은 휠 사전값으로 통제한 창고 2D–3D 비교는 이번 조사 범위의 문헌과 업계 자료에서 찾지 못했다. 이 공백을 메우는 것이 오히려 이 보너스의 기여다. 그 위에 우리 시스템 고유의 발견 하나를 실험으로 확정할 수 있다. 현행 2D 위치추정은 좁은 통로의 축 방향 구속을 통로 끝 너머의 시야에 기대고 있고, 교통 관리자가 통로 입구에 대기 로봇을 세우는 순간 그 시야가 사라진다. 3D의 가치는 "더 정확한 센서"가 아니라 "2D가 기대던 가정이 깨질 때 버티는 센서"로 측정될 때 가장 정직하게 드러난다.
