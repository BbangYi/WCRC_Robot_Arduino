# MissionRouteTuner

`Motor.ino` 미션 코드를 직접 바꾸지 않고, EEPROM 자세/짧은 주행/집기/배치 시퀀스를 현장에서 반복 테스트하는 Serial 콘솔입니다.

USB Serial Monitor를 기준으로 사용합니다. 블루투스 UART는 입력 보조 경로로만 둡니다.
기본값은 USB `Serial` 115200 baud, 블루투스 `Serial2` 9600 baud input-only입니다.
`Serial1`은 Dynamixel 모터 통신에 사용하므로 블루투스에 쓰면 안 됩니다.
블루투스로 스케치 업로드는 하지 않고, 긴 출력은 USB Serial Monitor에서 확인합니다.
모듈이 `Serial3` 또는 다른 속도로 연결된 보드라면 `Debug.h`의 `BLUETOOTH_SERIAL`, `BLUETOOTH_BAUD_RATE`만 바꾸세요.
미션 단계 이동값은 이 스케치의 `MissionConfig.h`에 있으며, 경기 코드 `Motor/MissionConfig.h`와 맞춰야 합니다.

## 기본 원칙

- 시작하자마자 자동으로 움직이지 않습니다.
- 모든 동작은 `mission start` 또는 명시적 Serial 명령을 입력해야 실행됩니다.
- 미션 흐름을 따라 확인하려면 `mission start` 후 SW1을 누릅니다. `mission start`가 SW1 단계 진행 모드를 자동으로 켭니다.
- 미션지시존에서 멈추지 않고 적재함 접근까지 묶어 확인하려면 `mission start quick` 또는 `mission run storage`를 사용합니다.
- `mission next`, `mission button on`, `mission block next`는 Serial 보조/fallback 명령으로만 남깁니다. 기본 현장 흐름은 SW1입니다.
- 명령을 외우지 않아도 됩니다. USB Serial Monitor에서 `guide`를 입력하면 현장 최소 명령만 나옵니다.
- `profile safe|normal|fast|max`로 속도/거리/튜닝 변화량 한도와 미션 속도 preset을 바꿉니다.
- `speed status|reset|set`으로 미션 단계별 주행/정렬 속도를 확인하거나 임시 조정합니다. 팔/집게 시퀀스 시간은 기본 200ms로 고정합니다.
- `pose tune`은 EEPROM 기준값을 바로 저장하지 않고 1회 테스트만 합니다.
- `pose apply`와 `pose save`는 저장 후보만 만들고, 실제 EEPROM 저장은 `pose confirm`을 입력해야 수행됩니다.
- `replay`로 마지막 실행 명령을 다시 반복할 수 있습니다.

## 외우지 말고 쓰는 방법

USB Serial Monitor에 아래 명령을 입력하세요. 긴 주제별 설명은 `help ...`로 분리했습니다.

```text
guide
mission start
mission start quick
help pose
help pixy
help advanced
```

## 미션 단계별 확인 모드

전체 미션을 바로 자동 실행하지 않고, 미션 흐름을 따라 한 단계씩 확인하려면 아래처럼 사용합니다.

```text
mission start
SW1             # status + pose verify + initial + 최초 장애물 접근 + 좌측 정렬 + 선 밟기 final forward 후 정지
SW1             # 미션지시존 Pixy scan 후 정지
mission rescan  # 필요 시 반복
pixy watch all 30
mission accept  # 스캔 결과가 괜찮으면 다음 이동 허용
SW1             # 적재함 접근 + SL/FR 정렬 후 정지
SW1             # 목표 열까지 한 칸씩 이동, 목표 열이면 columnscan 후 정지
SW1             # 저장된 columnscan 판정 기준으로 집기만 실행 후 정지
SW1             # 미션수행존으로 이동해 배치 후 정지
SW1             # 다음 블록이면 적재함 재정렬, 끝이면 finish 준비
SW1             # finish 후진
```

- `mission start quick` 또는 `mission run storage`: 미션지시존 접근, SL 정렬 뒤 라인을 밟는 final forward, 주행 중 Pixy 스캔, queue 자동 수락, 적재함 접근/정렬까지 한 번에 묶어 실행합니다.
- quick 실행 중 주행 스캔에서 queue가 비면 정지 스캔을 한 번 fallback으로 실행합니다.
- `mission next`: Serial fallback입니다. 현재 SW1 단계의 기본 동작을 한 번 실행합니다.
- `mission rescan`: 미션지시존 Pixy queue를 다시 구성하고 이동하지 않습니다.
- `mission accept`: 스캔 queue를 수락하고 다음 SW1에서 적재함 이동을 허용합니다.
- `mission button on/off`: SW1 단계 진행 수동 토글입니다. `mission start`가 자동으로 ON 처리합니다.
- 기본 queue는 미션지시존 Pixy 스캔 결과를 기반으로 구성하고, source 순회는 `CFG.storageRack.pickSlotOrder`, 배치는 `CFG.mission.goalPositions`를 사용합니다.
- 적재함 열 이동은 `1/5 -> 2/6 -> 3/7 -> 4/8` 기준입니다. 한 번의 `mission next`는 최대 한 열만 이동하고 멈춥니다.
- 현재 열 기준이 틀어졌으면 `mission column <1~4>`로 수동 보정합니다.
- `mission columnstep <mm> <mm/s>`로 한 열 이동량과 속도를 현장에서 숫자로 테스트합니다.
- `mission columnscan`으로 현재 열 Pixy storage 스캔을 실행하고 `signature/sourceSlot/goalSlot/pickLayer` 판정을 저장합니다.
- `mission align <sl> <fl> <fr> [tol]`은 적재함 1열을 처음 보는 스캔 기준 위치입니다. 현재 기본값은 `mission align 354 266 269 8`입니다.
- `mission gripalign upper <sl> <fl> <fr> [tol]`은 상층 집기 직전 더 깊이 들어가는 기준 위치입니다. 현재 기본값은 `mission gripalign upper 359 349 363 8`입니다.
- `mission gripalign lower <sl> <fl> <fr> [tol]`은 하층 집기 직전 기준 위치입니다. 현재 기본값은 `mission gripalign lower 354 325 337 8`입니다.
- `mission gripalign upper|lower run`으로 현재 위치에서 층별 집기 직전 PSD 정렬만 따로 실행할 수 있습니다.
- `mission placealign <sl> <fr> [tol]`은 미션수행존에서 내려놓기 직전 SL+FR 기준 위치입니다. 현재 기본값은 `mission placealign 635 220 8`입니다.
- `mission placealign run`으로 현재 위치에서 배치 SL+FR 정렬만 따로 실행할 수 있습니다.
- `mission undo`는 가능한 마지막 고정 거리 열 이동만 반대 방향으로 실행합니다. PSD 정렬, 장애물 접근, 집기, 배치는 자동 원복하지 않습니다.
- `mission instruction <sl> <ms> <raw>`는 미션지시존 SL 정렬 뒤 선을 밟는 final forward 값을 조정합니다. 기본은 `mission instruction 640 500 120`입니다. `ms=0`이면 끕니다.
- 기본값으로 돌아가려면 `mission auto`를 입력합니다.
- `mission upper` / `mission lower`: 필요할 때만 적재함 집기 분기를 수동 변경합니다.
- `mission slot <1~8>`: 필요할 때만 미션수행존 배치 칸을 수동 변경합니다.
- 감지 queue의 모든 블록을 처리하면 `mission finish` 또는 다음 SW1에서 현재 위치 기준 후진 finish를 실행합니다.
- `mission block next`: 숨김 fallback입니다. 기본 흐름에서는 사용하지 않습니다.
- `mission goto <stage>`: 필요한 단계로 직접 이동합니다. 예: `mission goto scan`, `mission goto column`, `mission goto place`.
- 각 단계 사이에서는 `pose tune`, `pixy watch`, `grip test`, `drive trim` 같은 일반 튜너 명령을 자유롭게 실행할 수 있습니다.

## 자주 쓰는 명령

```text
guide
mission start
mission start quick
mission run storage
mission next
mission rescan
mission accept
mission status
mission auto
mission undo
mission finish
status
pose verify
export json
profile normal
speed status
speed fast
speed set position 220
speed set psd 260
mission scanrate 500 10
mission scanrate 200 10
mission instruction 640 500 120
mission columnstep 72 150
mission columnscan
help pose
help pixy
help advanced
help speed
help math
pose list
pose backup
pose present
pose diff 3
pose plan 3 300
pose run 1 300
pose tuneplan 1 300 0 -30 +20 0
pose tune 1 300 0 -30 +20 0
pose apply initial_low
pose confirm
pixy scan
pixy sig 3 5
pixy watch 3 30
pixy fps
pixy brightness 80
pixy storage lower 10 0
pixy storage all 10 80
pixy sweep all 4 40 140 20 12 80
grip cycle 2 500
grip test 100 650 2 500
seq storage
seq pick upper
seq place 3
seq placeall
delaytest 2 7 300 3
drive back 300 120
drive trim 후진 200 100 1.00 0.95
replay
stop
!
```

## 도움말 주제

Serial Monitor에서 아래처럼 입력하면 현장 기준 설명이 출력됩니다.

```text
help pose
help drive
help seq
help pixy
help grip
help profile
help speed
help math
help risk
```

핵심 값의 의미는 다음과 같습니다.

- `pose run <id> <ms>`: EEPROM에 저장된 자세를 지정 시간에 실행합니다. `ms`가 작을수록 빠릅니다.
- `pose present`: 현재 팔 모터 1~4 raw 값을 출력합니다.
- `pose diff <id>`: 현재 자세와 EEPROM 자세의 차이를 출력합니다.
- `pose plan <id> <ms>`: 실제 이동 없이 변화량과 보정 시간을 미리 봅니다.
- `pose verify`: 현재 테스트에 필요한 자세 `1,3,4,5,7~14`가 저장되어 있는지 확인합니다.
- `pose tuneplan <id> <ms> <m1> <m2> <m3> <m4>`: `pose tune` 전에 목표 raw와 위험도를 미리 봅니다.
- `pose tune <id> <ms> <m1> <m2> <m3> <m4>`: EEPROM 기준값을 읽고 임시 목표값으로 한 번 움직입니다. 저장하지 않습니다.
- `+20`, `-30`, `0`: EEPROM 기준값 대비 증감입니다.
- `=2048`: 기준값이 아니라 절대 raw 목표값입니다.
- `pose apply [desc]`: 마지막 `pose tune` 결과를 저장 후보로 올립니다.
- `pose save <id> <desc>`: 현재 실제 자세를 저장 후보로 올립니다.
- `pose restore <id> <m1> <m2> <m3> <m4> [desc]`: 백업 raw 값으로 복구 후보를 만듭니다.
- `pose confirm`: 저장 후보를 실제 EEPROM에 씁니다.
- `pose cancel`: 저장 후보를 버립니다.
- `drive back 300 120`: 300mm 후진을 120mm/s 목표로 실행합니다.
- `speed status`: 미션 단계별 실제 적용 주행/정렬 속도와 고정 actuator 시간을 출력합니다.
- `speed fast`: `profile fast`와 같은 속도 preset을 적용합니다.
- `speed set position 220`: 적재함 방향 거리 이동을 220mm/s 후보로 임시 조정합니다.
- `speed set psd 260`: PSD 정렬 속도를 260 raw 후보로 임시 조정합니다.
- `mission scanrate 500 10`: 미션지시존 signature queue 스캔 시간을 0.5초, 샘플 간격을 10ms로 임시 조정합니다.
- `mission scanrate 200 10`: 인식이 안정적일 때 거의 멈추지 않는 빠른 스캔 후보입니다.
- `mission instruction 640 500 120`: 미션지시존 SL 정렬 뒤 500ms 동안 raw 120으로 더 전진하며 선을 밟고, 그 사이 Pixy를 계속 샘플링합니다.
- `mission instruction 640 700 120`: 선을 더 확실히 밟아야 할 때 final forward 시간을 늘리는 후보입니다.
- `mission columnstep 72 150`: 적재함 열 이동량/속도 후보를 임시 조정합니다.
- `mission align 354 266 269 8`: 적재함 1열 스캔 기준 PSD 위치를 임시 조정합니다.
- `mission gripalign upper 359 349 363 8`: 상층 집기 직전 PSD 위치를 임시 조정합니다.
- `mission gripalign lower 354 325 337 8`: 하층 집기 직전 PSD 위치를 임시 조정합니다.
- `mission gripalign upper|lower run`: 층별 집기 직전 PSD 위치로만 이동해 그립 전 거리를 확인합니다.
- `mission placealign 635 220 8`: 미션수행존 내려놓기 SL+FR 위치를 임시 조정합니다.
- `mission placealign run`: 배치 SL+FR 위치로만 이동해 내려놓기 전 거리를 확인합니다.
- `pixy scan`: Pixy가 현재 보는 signature, x/y, 면적을 출력합니다.
- `pixy sig <1~7>`: 특정 signature만 확인합니다.
- `pixy watch <1~7|all>`: 여러 프레임의 인식률, 평균 위치, 면적 흔들림을 계산합니다.
- `pixy storage [lower|all|upper] [frames] [minArea]`: 적재함 블록의 `x/y`가 pickup region `upper/lower` 안에 있는지 출력합니다. 행을 생략하면 lower 기준으로 봅니다.
- `pixy align [lower|upper|all] [timeoutMs] [minArea] [signatureMap]`: 블록을 upper/lower grip boundary 중심으로 조금씩 정렬하며 걸린 시간을 출력합니다.
- `pixy sweep [1~7|all] [expected] [start] [end] [step] [frames] [minArea]`: brightness 후보를 훑어 면적 기준을 넘은 블록 개수를 가장 안정적으로 찾는 값을 추천합니다.
- `pixy brightness <0~255>`: 카메라 밝기/노출 후보를 테스트합니다.
- `pixy fps`: 현재 FPS를 읽습니다. FPS가 낮으면 조명 부족 가능성이 큽니다.
- `grip cycle`: 집게 열기/닫기를 반복해 서보 동작을 확인합니다.
- `grip set <0~1000>`: 집게 서보 위치를 직접 보냅니다. 저장값은 바꾸지 않습니다.
- `grip test [open close] [repeat] [holdMs]`: 열림/닫힘 후보값을 반복 테스트합니다.
- `drive trim 후진 200 100 1.00 0.95`: 후진 200mm를 100mm/s로 실행하되 오른쪽 바퀴 목표량을 5% 줄입니다.
- `!`: 동작 중 긴급정지입니다. 모바일베이스를 정지하고 팔은 현재 위치를 목표로 hold합니다.

## 미션 자세 ID

- `1`: 초기 / 미션 지시존 카메라 자세
- `2`: 예비 / 수동 테스트
- `3`: 적재함 보기 / 안전 이동 자세
- `4`: 상층 집기
- `5`: 하층 집기
- `6`: 예비 / 수동 테스트
- `7~14`: 미션수행존 1~8칸 배치 자세

## 모터 값과 높이 조정

- `m1`: 하부 좌우 회전, raw `0~4095`
- `m2`: 어깨 링크, raw `1024~3474`
- `m3`: 팔꿈치 링크, raw `211~2640`
- `m4`: 손목/말단 링크, raw `722~3161`

`4096 raw = 360도`라서 `1 raw`는 약 `0.088도`입니다.
다만 실제 높이는 `m2~m4`가 함께 만드는 링크 자세로 결정됩니다. 한 모터만 크게 바꾸면 높이뿐 아니라 전후 거리와 말단 각도도 같이 바뀔 수 있습니다.

높이를 조정할 때는 먼저 `20~50 raw` 단위로 나누어 확인하세요.

```text
pose present
pose diff 3
pose tuneplan 3 300 0 +30 -30 0
pose tune 3 300 0 +30 -30 0
pose tune 7 300 0 0 +20 -20
```

움직임 방향은 조립/오프셋 상태에 따라 체감이 달라질 수 있으므로, 첫 테스트는 반드시 낮은 속도와 작은 변화량으로 확인해야 합니다.

## 미션 속도 조정

미션 단계가 너무 느리면 먼저 전체 preset을 올립니다.

```text
speed status
speed fast
mission start
SW1
```

특정 단계만 답답하면 부분 조정합니다.

```text
speed set front 330
speed set slow 200
speed set psd 280
speed set position 240
mission columnstep 72 150
```

- `front` / `slow`: 미션지시존 전면 접근 속도입니다. 단위는 velocity raw입니다.
- `psd`: SL/FR PSD 정렬 속도입니다. 단위는 velocity raw입니다.
- 미션지시존이 너무 오른쪽이면 `MissionConfig.h`의 `CFG.psd.missionSl`을 올립니다. 현재 기본값은 `640`입니다.
- `position`: 적재함 쪽 거리 이동 속도입니다. 단위는 `mm/s`입니다.
- 팔 자세/집게 hold 시간은 미션 튜너에서 200ms로 고정합니다. 큰 raw 이동도 profile 상한 때문에 최대 500ms 안에서 실행됩니다.
- `speed reset`: 현재 profile 기준 preset으로 되돌립니다.

너무 빠르게 올리면 PSD 정렬 오버슈트, 블록 밀림, 팔 자세 흔들림이 생길 수 있습니다. `profile fast`에서 안정되면 `speed set`으로 한 값씩만 올리세요.

## 적재함 Pickup Region 보정

Motor는 적재함에서 signature만 따라가지 않고, Pixy 블록 중심점이 설정한 pickup region 안에 들어왔는지 보고 집습니다.

```text
pixy storage lower 10 0
pixy storage all 10 80
```

- 적재함 전체 boundary: `(94,4)~(235,205)`에 margin 2를 둡니다.
- upper pickup/grip boundary: `(129,57)~(180,110)`.
- lower pickup/grip boundary: `(139,155)~(178,207)`.
- `lower`: lower boundary만 봅니다.
- `all`: upper/lower 전체 판정을 확인합니다.
- `pixy align`은 boundary 안에 들어오는 것에서 끝내지 않고, boundary 중심점 기준 `centerToleranceX/Y` 안으로 들어올 때까지 보정합니다.
- 미션 자동 집기 흐름은 `mission gripalign upper|lower` PSD 위치로 먼저 붙은 뒤 `pixy align`으로 x/y 중심을 미세 조정합니다.

정렬 시간 테스트:

```text
pose run 3 300
pixy storage lower 10 0
pixy align lower 5000 0
```

- `pixy align`은 실제로 모바일베이스를 조금씩 움직입니다.
- 출력되는 `elapsedMs`가 안정적으로 나오면 `storageGripTarget.alignTimeoutMs`를 그보다 조금 크게 잡습니다.
- 타임아웃이 나면 Motor 미션에서는 해당 블록을 스킵하고 다음 블록으로 넘어갑니다.

## 위험 제한 기준

`profile`별로 한 번에 허용하는 raw 변화량, 주행 거리, 주행 속도, 예상 주행 시간, 미션 속도 preset이 제한됩니다.
큰 변화는 한 번에 처리하지 말고 여러 번 나누어 테스트하세요.

하드웨어 연결 후 첫 사용은 반드시 바퀴를 띄운 상태에서 `drive` 명령부터 확인하세요.

`profile max`는 가장 빠른 테스트를 위한 모드입니다. 블록 근처, 팔이 낮은 상태, 좁은 구간에서는 사용하지 말고 바퀴를 띄우거나 빈 경기장에서만 확인하세요.

## Pixy / 집게 / 자세 점검 순서

미션 실패는 보통 `인식 실패`, `접근 자세 불량`, `집게 실패`, `배치 자세 불량` 중 하나입니다.
아래 순서로 나누어 보면 원인을 빠르게 분리할 수 있습니다.

```text
pose verify
pose present
pixy lamp off
pixy scan all 5
pixy sig 1 5
pixy watch 1 30
pixy fps
pixy sweep all 4 40 140 20 12 80
grip cycle 2 500
grip test 100 650 2 500
seq storage
seq pick upper
seq place 1
```

`seq pick upper`는 RFID 카드 간섭을 줄이기 위해 상층 그립과 storage 복귀를 `m4 -> m3 -> m2` 순서로 나눠 실행합니다. 하층 집기는 기존처럼 한 번에 움직입니다.

### 충분한 부분

- EEPROM 기준 자세를 읽고, 실행하고, 조정하고, 저장하는 흐름은 들어 있습니다.
- 실제 저장 전 `pose plan`, `pose tuneplan`, `pose diff`로 변화량을 확인할 수 있습니다.
- Pixy signature 1~7 인식 결과를 Serial로 볼 수 있습니다.
- `pixy watch`로 인식률과 흔들림을 수치로 확인할 수 있습니다.
- `pixy sweep`으로 brightness 후보를 자동 비교하고 추천값을 출력할 수 있습니다. 같은 signature 블록이 여러 개 있어도 각각 개수로 계산합니다.
- 집게 열기/닫기/반복 테스트를 미션 전체 실행 없이 확인할 수 있습니다.
- `grip set`, `grip test`로 블록 두께에 맞는 집게 후보값을 EEPROM/상수 변경 없이 확인할 수 있습니다.
- `seq placeall`로 미션수행존 1~8칸 배치 자세를 그리퍼 동작 없이 연속 확인할 수 있습니다.
- 동작 중 `!` 긴급정지를 사용할 수 있습니다.
- 위험한 큰 raw 변화나 긴 주행은 profile 제한으로 막습니다.

### 아직 부족한 부분

- 집게가 실제로 블록을 잡았는지는 서보 피드백이 없어서 직접 감지하지 못합니다.
- Pixy 인식 범위 자체를 Arduino 코드에서 새로 학습시키는 것은 하지 않습니다. PixyMon에서 학습된 signature 1~7을 필터링해서 쓰는 구조입니다.
- 적재함 4x2 칸의 실제 좌표를 자동으로 보정하는 기능은 아직 없습니다. 현재는 pose 7~14와 `seq place`로 칸별 배치 자세를 검증하는 단계입니다.
- 후진 휨 보정 비율을 미션 코드에 자동 반영하지는 않습니다. `drive trim`으로 검증한 뒤 별도 단계에서 `Motor` 설정에 반영하는 것이 안전합니다.
- 실제 경기 중 실시간 자동 캘리브레이션은 아직 하지 않습니다. 경기 코드는 안정성이 더 중요하므로, 튜너에서 검증한 값만 `Motor.ino`/EEPROM에 반영하는 방식이 좋습니다.

## 제약사항

이 튜너는 현장 캘리브레이션을 빠르게 하기 위한 도구입니다. 경기 미션 코드의 안정성을 깨지 않기 위해 아래 항목은 자동으로 바꾸지 않습니다.

- `Pins.h`, Dynamixel ID, 모터 방향 같은 하드웨어 연결 정의는 건드리지 않습니다.
- `Motor.ino`의 미션 순서와 실제 주행 로직은 이 튜너에서 직접 수정하지 않습니다.
- Pixy signature 학습은 PixyMon에서 수행합니다. Arduino 쪽에서는 이미 저장된 signature를 읽고, brightness/lamp/filter/watch/sweep만 조정합니다.
- Pixy PRM 파일을 경기 중 자동으로 교체하는 방식은 사용하지 않습니다. 최종 PRM 하나를 경기 전에 로드하고 검증하는 쪽이 더 안전합니다.
- EEPROM 자세 저장은 `pose apply`, `pose save`, `pose restore`로 후보를 확인한 뒤 `pose confirm`을 입력했을 때만 수행합니다.
- `drive trim`, `grip test`, `pixy sweep` 결과는 후보값입니다. 튜너가 `MissionConfig.h`, `Gripper.h`, `Motor.ino`에 자동 반영하지 않습니다.
- Notion이나 사이트 운영 기록은 경기 전/후 관리용입니다. 주행 중 제어 루프가 외부 사이트나 네트워크에 의존하면 안 됩니다.

## 제한사항과 어려운 점

- Pixy는 색 기반 인식이라 조도, 그림자, 반사, 배경색, 카메라 각도에 영향을 받습니다. `pixy sweep`은 brightness 후보를 추천할 뿐, 잘못 학습된 signature나 배경 간섭을 해결하지는 못합니다. 한 블록이 여러 조각으로 쪼개져 보이면 개수가 부풀 수 있으므로 `pixy scan`으로 area와 크기를 같이 봐야 합니다.
- 미션 지시존과 적재함의 조도가 다르면 같은 signature라도 인식률이 달라질 수 있습니다. 이 경우 먼저 조명/거리/각도/차폐판을 고정하고, 마지막에 PixyMon signature를 다시 조정해야 합니다.
- 집게에는 힘/전류/접촉 피드백이 없으므로 “잡았다”를 코드가 직접 알 수 없습니다. `pixy watch`, 접근 pose, `grip test`를 나눠서 실패 원인을 분리해야 합니다.
- EEPROM 자세가 잘못 저장되어 있으면 시퀀스가 논리적으로 맞아도 실제 팔이 위험한 위치로 갈 수 있습니다. `pose verify`, `pose backup`, `pose plan`, `pose diff`를 먼저 확인해야 합니다.
- 후진 휨은 바닥 마찰, 배터리, 바퀴 장착, 무게중심, 개별 모터 상태가 같이 영향을 줍니다. `drive trim`은 현상 분리를 위한 테스트이고, 한 번의 비율값이 모든 환경에서 항상 맞는 것은 아닙니다.
- 저장된 4x2 적재함 위치와 미션수행존 1~8칸 배치는 현재 pose 기반 검증 단계입니다. 카메라 좌표로 슬롯 중심을 자동 보정하는 기능은 아직 별도 구현이 필요합니다.

## 테스트 결과 기록 방식

현장 테스트는 기억에 의존하면 값이 쉽게 섞입니다. Arduino는 Mac 파일을 직접 쓰지 못하므로 튜너가 Serial에 출력하는 JSON 한 줄 로그를 Mac에서 `Motor/ops/local-tuning-runs.jsonl`로 저장합니다.
EEPROM pose 계약/검증 상태는 `Motor/ops/eeprom-poses.json`에만 두고, 임시 테스트값은 local jsonl에 남깁니다.

```text
export json
{"type":"mission-queue","count":4,"items":[...]}
{"type":"mission-columnstep","stepMm":72.00,"speedMmPerSec":150}
{"type":"mission-columnscan","blockIndex":1,"sourceSlot":1,"goalSlot":1,"targetSignature":2,"column":1,"stepMm":60.00,"speedMmPerSec":80}
{"type":"mission-columnscan-decision","found":true,"blockIndex":1,"sig":2,"sourceSlot":1,"goalSlot":1,"pickLayer":"upper","pickupRegion":1,"x":152,"y":48,"area":420}
```

`export json`은 EEPROM pose 1~14, 필수 pose 검증 대상, 현재 pose, profile/speed, mission queue, 마지막 columnscan 판정, MissionConfig 주요 값, Pixy 현재 프레임, USB/BT Serial 설정을 한 줄 JSON으로 출력합니다.
Serial Monitor 출력에서 JSON 줄만 복사하거나 별도 터미널 로거를 붙여 저장하세요. 사람이 읽는 메모는 `Motor/ops/events.jsonl` 또는 별도 노트에 남겨도 됩니다.

```text
2026-04-30 18:20, mission-zone, wcrc-final.prm, brightness=80, lamp=on, pixy sweep all expected=4 -> best=80 fullFrames=9/12, 성공
2026-04-30 18:32, storage, sig=3, watch 30 -> seen=28 avgX=142 avgY=78 areaRange=210, 조도 안정
2026-04-30 18:45, pose 7, tune 0 +20 -20 0, apply 전, 높이 3mm 낮음
2026-04-30 18:55, drive trim 후진 200 100 1.00 0.95, 오른쪽 휨 감소, 재테스트 필요
```

Git commit과 연결하려면 테스트가 끝난 뒤 `git rev-parse --short HEAD` 값을 같이 적으면 됩니다. 나중에 사이트 대시보드나 Notion 요약을 붙이더라도, 원본 기록은 Git-tracked 파일에 남기는 쪽이 가장 안전합니다.

## 다음 발전 방향

지금 단계에서 바로 `Motor.ino`를 크게 바꾸기보다, 튜너로 값을 검증한 뒤 작은 단위로 반영하는 순서가 안전합니다.

1. `pixy sweep`으로 찾은 장소별 brightness 후보를 비교하고, 최종 한 값을 `MissionConfig.h`에 명시합니다.
2. `Motor.ino`에는 단일 프레임 인식 대신 `N프레임 중 M프레임 이상`, `면적 최소값`, `x/y 흔들림 제한`을 통과한 블록만 집도록 안정 판정 함수를 추가합니다.
3. 적재함 4x2 슬롯은 먼저 고정 좌표와 signature 매핑을 기록하고, 이후 Pixy x/y 중심값이 예상 슬롯 범위 안에 있는지 검증하는 방식으로 확장합니다.
4. 집게 실패가 반복되면 `grip test` 후보값을 기준으로 열림/닫힘 상수를 조정하되, 실제 저장 전 블록 두께별 반복 테스트를 남깁니다.
5. 후진 복귀는 현재 잘 맞는 시간값을 유지하고, 짧은 거리 `drive trim` 결과가 반복적으로 같은 방향으로 확인될 때만 미션 설정에 반영합니다.

## 반복 개선 루프

문제가 보일 때마다 바로 `Motor.ino`를 고치면 성공했던 흐름이 같이 깨질 수 있습니다. 아래 순서로 분리해서 처리하는 것을 기본으로 둡니다.

```text
help advanced
pose backup
pixy watch all 30
pixy sweep all 4 40 140 20 12 80
```

### 바로 코드에 반영하지 않는 값

- 한 번만 성공한 빠른 주행 속도나 짧은 delay
- 조명이 흔들리는 상태에서 얻은 Pixy brightness/sweep 결과
- 블록이 쪼개져 보이는 상태에서 얻은 expected count 결과
- 손으로 보정해서 우연히 된 pose 값
- 핀, Dynamixel ID, 모터 방향, PRM 교체 방식

### 튜너에서 반복 검증할 값

- `pose tuneplan`/`pose tune`으로 확인한 높이와 접근 자세
- `grip test`로 확인한 열림/닫힘 후보값
- `drive trim`으로 확인한 좌우 또는 개별 바퀴 보정 후보
- `pixy watch`/`pixy sweep`으로 확인한 조도 후보
- `seq placeall`에서 칸별로 어긋나는 pose 7~14

### Motor 코드에 반영해도 되는 기준

- 같은 조건에서 최소 3회 이상 같은 결과가 나옵니다.
- 실패 원인이 `Pixy`, `자세`, `집게`, `주행` 중 하나로 분리됩니다.
- 기존에 성공하던 단계가 깨지지 않는 작은 변경입니다.
- 변경 전 `pose backup` 또는 Git diff로 되돌릴 근거가 남아 있습니다.
- 경기 직전에는 이미 검증된 값만 반영하고, 새로운 실험값은 다음 테스트로 넘깁니다.

### 우선순위

1. 안전: 충돌, 과속, EEPROM 오염, 핀/ID 변경 가능성 제거
2. 정확도: Pixy 인식 안정도, 자세 높이, 집게 물림, 칸 위치
3. 재현성: 같은 명령을 여러 번 반복해도 같은 결과가 나오는지
4. 속도: 정확도가 확보된 뒤 delay와 주행 속도 단축
5. 운영 기록: 성공/실패와 Git commit, EEPROM pose 변경 내역 연결

## Pixy 조도 대응 아이디어

Pixy2 CCC는 색상의 hue/saturation 중심으로 빠르게 블록을 찾기 때문에 속도는 좋지만, 실제 경기장에서는 조명 방향, 반사, 그림자, 배경색 때문에 안정도가 흔들릴 수 있습니다.

권장 방향은 코드에서 signature를 계속 바꾸는 것이 아니라, **환경을 안정화하고 인식 품질을 수치로 확인하는 구조**입니다.

1. Pixy LED를 켜고/끄는 두 조건을 모두 `pixy watch`로 비교합니다.

```text
pixy lamp on
pixy watch 1 30
pixy fps
pixy lamp off
pixy watch 1 30
pixy fps
```

2. 밝기 후보를 자동으로 훑어 추천값을 찾습니다.

```text
pixy sweep all 4 40 140 20 12 80
pixy watch all 30
```

`pixy sweep all 4 40 140 20 12 80`의 의미:

- `all`: signature 1~7 전체 후보
- `4`: 기대 블록 개수. 미션지시존처럼 4개를 찾고 싶으면 4. 같은 signature가 여러 개여도 블록별로 계산
- `40 140 20`: brightness 40부터 140까지 20 간격으로 테스트
- `12`: 밝기마다 12프레임 확인
- `80`: area가 80보다 작은 잡음은 무시

모든 블록을 찾은 brightness가 없으면, 가장 많은 블록을 잡은 후보를 추천합니다.

3. 추천 brightness를 기준으로 각 signature 인식률이 80% 이상인지 확인합니다.

```text
pixy watch 1 30
pixy watch 2 30
pixy watch 3 30
```

4. 배경과 블록 색이 비슷하면 PixyMon에서 signature를 다시 학습합니다. Arduino에서는 PixyMon에 저장된 1~7번 signature를 필터링해서 쓰는 쪽이 안전합니다.

5. 색 하나만으로 헷갈리면 블록에 작은 색상 조합 마커를 붙이는 `color code` 방식이 대안입니다. 이 방식은 7개 signature 제한을 우회하고 오인식 가능성을 줄일 수 있지만, 실제 블록에 마커를 붙일 수 있어야 합니다.

6. 미션 지시존/적재함에서 같은 색이 다른 조도로 보이면 “다른 signature로 새로 학습”하기보다 먼저 조명, 카메라 각도, 배경 차폐판, 거리 고정을 우선합니다. signature를 너무 많이 쪼개면 7개 제한에 빨리 걸립니다.

현재 튜너의 `pixy watch`는 이 판단을 위한 최소 측정 도구입니다. 인식률, 평균 위치, 면적 흔들림이 안정적으로 나오면 그 조건을 미션 코드에 반영하는 것이 좋습니다.

## 후진 휨 보정 테스트

`drive trim`은 미션 코드의 주행 로직을 바로 바꾸기 전에, 바퀴 목표 회전량 보정 비율을 짧은 거리에서 시험하는 명령입니다.

```text
profile safe
drive back 100 80
drive back 200 100
drive trim 후진 200 100 1.00 0.95
drive trim 후진 200 100 0.95 1.00
```

좌/우 보정:

```text
drive trim <방향> <mm> <mmPerSec> <leftScale> <rightScale>
```

개별 바퀴 보정:

```text
drive trim <방향> <mm> <mmPerSec> <fl> <fr> <bl> <br>
```

`1.00`은 기본값, `0.95`는 목표 회전량 5% 감소, `1.05`는 5% 증가입니다.
보정 비율은 안전을 위해 `0.70~1.30` 범위만 허용합니다.

후진이 오른쪽으로 휘는 경우에도 원인이 항상 “오른쪽 바퀴가 빠르다”로 고정되지는 않습니다.
바닥 마찰, 바퀴 장착 방향, 좌우 하중, 모터 방향 설정이 섞일 수 있으므로 `1.00 0.95`와 `0.95 1.00`을 짧은 거리에서 비교해서 실제로 직진성이 좋아지는 쪽을 선택하세요.

## 계산식

자세 실행/튜닝은 가장 많이 변하는 모터 raw 변화량을 기준으로 자동으로 시간을 보정합니다.

```text
pose run  maxRawDelta = max(|목표 m1~m4 - 현재 m1~m4|)
pose tune stepMaxRawDelta = max(|목표 m1~m4 - 현재 m1~m4|)
pose tune eepromMaxRawDelta = max(|목표 m1~m4 - EEPROM m1~m4|)  # 저장 전 확인용
safeMs = min(profile.maxPoseMs, max(요청ms, profile.minPoseMs + maxRawDelta * msPerRawTick))
```

주행은 바퀴 반지름 `30mm`와 Dynamixel 1회전 `4095 raw`를 기준으로 변환합니다.

```text
wheelRaw = distanceMm / 30mm * 4095 / (2*pi)
driveMs = distanceMm * 1000 / mmPerSec
```

옴니휠 X/Y 이동은 내부적으로 아래와 같은 선형 모델로 확장할 수 있습니다.

```text
k = 4095 / (2*pi*30)
[fl, fr, bl, br] = k * [y+x, y-x, y-x, y+x]
```

현재 명령은 `forward/back/left/right` 축 방향 테스트만 제공합니다. 슬롯 좌표 기반 자동 보정은 이 식을 이용해 별도 단계로 확장하는 것이 안전합니다.
