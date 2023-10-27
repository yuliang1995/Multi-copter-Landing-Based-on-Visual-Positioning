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
接著相機可以透過ArUco取得相機的姿態估測，在這之後我們需要在實驗場所中建立一個世界座標系統，這時我們會有相機座標、ArUco 座標與世界座標，而三者關係，如下圖。  
![image](https://github.com/yuliang1995/Multi-copter-Landing-Based-on-Visual-Positioning/blob/main/image/%E5%BA%A7%E6%A8%99%E7%B3%BB%E7%B5%B1.jpg?raw=true)  
我們可以透過這三者關係去做座標轉換，轉換過後就可以取得相機在世界座標上的位置，達到室內定位效果，接著我們就可以控制無人機，並在我們所指定的範圍內降落。
