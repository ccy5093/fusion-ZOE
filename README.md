# Sensorfusion
A basic sensor-fusion project based on Zoe Cars with MATLAB  
可以通过reader中的Question A B C D了解到项目内不同的无人车传感器融合问题  

Problem A:   
The goal is to improve the localization of each vehicle by doing the data fusion of its Ublox with its DR sensors in a loosely coupled way  

Problem B:  
The vehicles communicate and the leader sends only its estimated pose with the associated covariance matrix (from their EKF).  
The goal is to improve the localization of the follower by doing the data fusion of its DR sensors with its own Ublox and the received pose of the leader translated into its own body frame thanks to the estimated relative pose obtained from the lidar measurements.  

Problem C:  
This problem is the same as Problem B but we want to test more consistent oplus and ominus functions by using Unscented Transformation (UT)  

Problem D:  
The vehicles communicate and still exchange their estimated pose with the associated covariance matrix but now the follower sends also its measured relative pose (done by the lidar). The goal is to improve the localization of the two vehicles by doing the data fusion of all the sensors.
