0. 준비
  가. mw_ahrs 기본 ss 설정은 0
  나. ros 패키지를 사용하려며 ss를 7로 변경
  다. 7은 가속도, 각속도, 각도 데이터

1. 설정 순서
  가. spider
    1) ss=0->7 : 데이터 전송 중단->가속도, 각속도, 각도 데이터 요청(시리얼)
    2) cmd=1 : 플래시 메모리 저장
    3) gs=3->0 : 2000dps->250dps
    4) as=3->0 : 16g->2g