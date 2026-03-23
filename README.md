## 📌 프로젝트 개요

본 프로젝트는 창고 환경에서 다수의 AMR(Autonomous Mobile Robot)이  
충돌 없이 목표 지점까지 이동하는 과정을 시뮬레이션한 프로그램이다.

- 각 로봇은 A* 알고리즘을 이용해 최단 경로를 생성
- 이동 중 충돌 가능 상황이 발생하면 경로를 재탐색
- 우선순위 기반으로 충돌을 회피하며 최종 목표 도달

---


## 📷 미리보기

<p align="center">
  <img src="assets/start1.png" width="45%">
  <img src="assets/arrive1.png" width="45%">
</p>

### 최단거리 경로를 이용한 시뮬레이션 결과
왼쪽: A* 기반 경로 계획(Path Planning)  
오른쪽: 계획된 경로를 따른 자율 주행 및 목표 도달


<p align="center">
  <img src="assets/start2.png" width="45%">
  <img src="assets/arrive2.png" width="45%">
</p>

### 다중 로봇 환경에서 시뮬레이션 결과
다중 AMR 환경에서 각 로봇이 독립적으로 경로를 생성하며 이동

<p align="center">
  <img src="assets/action3.gif" width="80%">
</p>

### 다중 로봇 환경에서 시뮬레이션 결과(회피, 경로 재탐색)
다중 로봇 환경에서 충돌 회피 및 경로 재탐색을 통해 모든 로봇이 목표에 도달
