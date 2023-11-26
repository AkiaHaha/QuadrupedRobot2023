# <2023>
## Git Test:
* hello, this is a push from windows 223.10.27
* hello, this is a push from Linux kaanh2, 2023.10.27 17:24
* wakaka!!!

## E4->10.30:
* Add a function of 12 motors drive separately ; MotorTest12() ;
* Modify the function createControllerROSMotorTest() for adapting to the motor number ; and also other basic settings in the function ;
* output the final Body PE & Leg Point using FwdKin of Matrix_28 ;
* test them successfully with virtual motor ;![Alt text](image-1.png)

## E5->11.7:
* Add a function for ellipse trajectory move for 4 legs; without gait planning;

## E6 ->11.10
* Use modified aris of function "Show" to show motor pos successfuly.![](https://secure2.wostatic.cn/static/kAaB3ZTpR8dXto3nabKnSR/image.png?auth_key=1699584100-ctqYQZrx8RP5oTxMSMSUDa-0-1a23ad489a492a72c78eff2b81ce3edd)

* Modified e4 and build success.

## E7 ->11.10
* USe std::vector to rebuild the whole ellipse plan function named EllipseTrajectory5; 
* Set a class use template named Matrix to directly modify 28D matrix startModelPE(4, 7) and centerPoint(4, 3);
* Use parameter list to transfer value in constructor function which improves efficiency;
* Plan it with mindmap first and finally  built successfully  with long time debug~
![](https://secure2.wostatic.cn/static/mhSyuebyMyhB4rkN2Hx2zA/image.png?auth_key=1699626773-2cetRpKWe1XwiMTYFcRK7p-0-e57fdd5c996658fca55f861569145811)
![](https://secure2.wostatic.cn/static/nbVyL5yhcf9hgDEct3dYHq/image.png?auth_key=1699626802-fTQU8VauJ7Vb4CoFHqgmLA-0-db5b7cf5bf358be88c588b291a652b29)

## E8 ->11.10
* A  new class to run  4-leg ellipse curve -> Ellipse4LegDrive2();
* Build successfully at 1:20 AM;

## E9 ->11.12
* A new function ReadInfoemation to play a part of Show(sh) in aris;
* But it has a much more easy to read format;
* Tested many times they have the same fun in REAL motor; in virtual motor, Mmodel Pos has a little problem;


## E10 ->11.13
* A new class to run  4-leg ellipse curve -> Ellipse4LegDrive3();
* And its corresponding Ellipse plan EllipseTrajectory7;
* Use std::vector<>  to arrange all the matrixs; index by cols * i + j;
* Correct a big mistaken: Ellipse plan, theta_ should start from pi; not 0;
* Built successfully at 23:40;

## E11 ->11.18
* Run quadruped ellipse curve success;
* command "zero" to set the motors to the position of initial model;
* command "t" to run single motors seperately;
* command "tt" to drive all  motors together that the robot's 4-Leg bent at the same time;
* all the command runs success but a little wired;

## E12 ->11.20
* Rewrite functions of Ellipse plan by simple array instead of std::vector