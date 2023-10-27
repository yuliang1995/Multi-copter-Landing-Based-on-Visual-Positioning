# 使用工具  
ArUco Marker  
Python3.7  
無人機 Tello  
棋盤格  
![image](https://github.com/yuliang1995/Multi-copter-Landing-Based-on-Visual-Positioning/blob/main/image/ARUCO.jpg?raw=true)  
![image](https://github.com/yuliang1995/Multi-copter-Landing-Based-on-Visual-Positioning/blob/main/image/%E6%A3%8B%E7%9B%A4%E6%A0%BC.jpg?raw=true)
# 作法
實驗場所為下圖  
![image](https://github.com/yuliang1995/Multi-copter-Landing-Based-on-Visual-Positioning/blob/main/image/%E5%AF%A6%E9%A9%97%E5%A0%B4%E6%89%80.jpg?raw=true)  

首先，使用棋盤格取得相機校正參數，參數在使用ArUco Marker定位時使用， ArUco Marker是一種可以透過程式自動產生內部編號的二維碼，而ArUco的教材網路上有許多可以參考。  
接著相機可以透過ArUco取得相機的姿態估測，在這之後我們需要在實驗場所中建立一個世界座標系統，這時我們會有相機座標、ArUco 座標與世界座標
