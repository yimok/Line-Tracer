# Line Tracer 

### 사용 부품 및 센서
- Cortext-M4 STM32F4 보드 2개 , 적외선 센서 4세트, 초음파 센서 1개 , 11.1V 배터리 1개 , 3.7V 배터리 1개 , Stepping Motor 1개
- 3축 가속도 센서 1개 , 7805 1개, DC-UPS(5v 승압모듈) 1개 , 블루투스v1.2 2개 

### 구현기능(리모트 컨트롤)

- 가속도 센서를 기판 중앙에 설치하고 , stm32 보드 1개와 블루투스를 장착 후 연결 , 3.7v 배터리를 5v 승압기에 연결하여 마이크로프로세서에 전압 인가
- 리모트 컨트롤 기울기에따라 총 10가지 동작가능 0~9 의 숫자를 블루투스 통신을 사용해 라인트레이서 마이크로프로세서쪽으로 전송 
- 앞으로 3단계 뒤로 3단계 좌우 각각 2단계 제자리 1단계

### 구현기능(라인트레이서)

- 기판위에 적외선센서 4개와 초음파센서, stm32보드, 블루투스 를 장착하고 연결 7805 로 12v 배터리의 전압을 변환하여 5v 를 각각 모터드라이브, 마이크로프로세서에 인가 하여 작동
- 모서리 부분의 적외선센서 4개로 벽의 거리를 센싱 하여 벽면이 가까워지는 쪽의 반대 바퀴의 RPM을 늦춤, 초음파 센서는 정면에 벽이 근접하면 멈춤
- 리모트 컨트롤로부터 얻은 0~9의 값에따라 모터의 좌,우 바퀴를 각각 개별적으로 타이머 돌리면서 구동시킴  


### 회로

![image 1](http://blogfiles.naver.net/MjAxNjEyMTVfMjky/MDAxNDgxODA5MzM2NjEy.5dbcDe1qOloNU0mO_eZedXkCIN97XuysrqA_rNBKYLUg.gW1Jjrs1iY40aefI3_Mpz0oT6Kq4cGK6hRNJzMK3etsg.PNG.dlagyrbs/%ED%9A%8C%EB%A1%9C.png) 

### 결과 사진

![image 2](http://postfiles16.naver.net/MjAxNjEyMTVfMjc2/MDAxNDgxODA5MzM1MTMw.ZgyHbhnxOIlm6LKKZDd1zFzs2pxKaB0wqlBdxoJ3_i8g.YttF_gbEmz3mQXGVW04v3xXdk8uUJHLTTwQgu1dKX2og.PNG.dlagyrbs/remote.png?type=w2) 

![image 3](http://postfiles3.naver.net/MjAxNjEyMTVfMzUg/MDAxNDgxODA5MzMzMzU3.gcUl_rRqs9FB9dwIQENyPhG61EDR4gVbPwqlZvtmCBUg.Zc103ZKD5Qm94G02Z2UNGL2j2OMks-PBJGNgosnIEP4g.PNG.dlagyrbs/lintracer.png?type=w2) 



### 결과 영상

|             설명       | 영상 주소               |
| ---------------------- | ------------------------ |
| 가속도 센서로 모터 제어               |https://youtu.be/CWzntotpnHU     |                                                                          
| 초음파 센서로 모터 제어                |https://youtu.be/7dy97rkTpmQ         |                                                                                                     
| 시연 영상               |https://youtu.be/vs4gfxyZsqw      |         

