/* Copyright
 * Copyright (c) 2012-2014 Aldebaran Robotics. All rights reserved.
 * Use of this source code is governed by a BSD-style license that can be
 * found in the COPYING file.
 */

/* Coautores
   Marco Ramirez
   Juan Carlos Aguilera Perez
   Aurelio Puebla
   Fernando Lopez
*/

/* Aspectos de revision
 * Confirmar se mantega dentro del carril
 * Revisar Caidas
 * Agregar que se detenga a los 3 mins
 * Crear rutina de apagado y encendido
*/

/* Reglas
 * Distancia total 5m
 * Franja inicio 18 cm
 * Franja final 30 cm y de color rojo
 * Max tiempo fuera de carril = 3 segs
 * Se considera fuera de carril con 1 pie fuera de la linea
 * Si algo sucede el robot tiene que poder responder autonomamente (levantar)
*/

#include <iostream>
#include "NaoVision.h"
#include "NaoMovement.h"

using namespace std;
using namespace AL;
using namespace cv;

int main(int argc, char *argv[]) {
    const int port = 9559;
    string ip = argv[1];        // NAO ip
    cout << "IP: " << ip << endl;

    bool LOCAL = false;         // Flap for the kind of execution (local or remote).
    bool NAO = true;
    bool finish = false;
    char key = 'x';
    double angleToBlackLine;    // Angle of the detected line.
    int yellowArea;
    int redArea;
    int finalArea;

    Mat bottomImage, topImage;
    NaoVision naoVision(ip, port, LOCAL);
    NaoMovement naoMovement(ip, port, LOCAL);
    VideoCapture cap(1);        // Class for video capturing from video files or cameras.

    naoMovement.initialPositionIndividualRace();

    while (key != 27 && !finish) {
        if (NAO) {
            bottomImage = naoVision.getImageFrom(NaoVision::BOTTOM_CAMERA);

        } else {
            cap >> bottomImage;
            naoVision.setSourceMat(bottomImage);
        }



        if (naoVision.naoIsNearTheGoalRelayRace(bottomImage)) {
            topImage = naoVision.getImageFrom(NaoVision::TOP_CAMERA);

            yellowArea = naoVision.getAreaYellowColor(topImage);
            cout << " Yellow Area : " << yellowArea << endl;
            finalArea = naoVision.FinalLineFilterRelayRace(bottomImage);
            cout << "Final Area : " << finalArea << endl;

            if(yellowArea >=12 && yellowArea< 40){
                cout<<"Finishing on left" << endl;
                naoMovement.rightCorrection();
                finish = true;
            }
            else {
                cout<<"Finishing on right" << endl;
                naoMovement.middleZoneRelayRace();
                finish = true;
            }

        } else {
            angleToBlackLine = naoVision.calculateAngleToBlackLine();
            naoMovement.moveInRelayRace(angleToBlackLine);
        }
        key = waitKey(10);

        for (int i = 0; i < 250000; i++);   // Delay.
    }

    naoVision.unsubscribe();
    naoMovement.stop();

    return 0;
}


/*
    naoMovement.initialPositionRelayRace();

    bool onNear = false;
    bool finish = false;
    bool naoDetected = false;
    //Caminado inicial Normal
    while (key != 27 && !naoDetected){
        if (NAO) {
            src = naoVision.getImageFrom(NaoVision::TOP_CAMERA);
        } else {
            cap >> src;
            naoVision.setSourceMat(src);
        }

        //naoVision.calibrateColorDetection();
        //naoVision.ColorFilter(src);
        yellowArea = naoVision.getAreaYellowColor(src);
        cout << "Yellow Area : " << yellowArea << endl;
        if(yellowArea > 10 && yellowArea < 40){
            cout << "Brace Your selfs" << endl;
            cout<<"Yellow Area : " << yellowArea << endl;
            naoDetected = true;
        }

            key = waitKey(10);
            for (int i = 0; i < 250000; i++);   // Delay.
    }

    naoMovement.initialPositionIndividualRace();

    //Second Loop Normal walk of individual race
    while (key != 27 && !finish){
        cout<< "Entered on the second loop"<< endl;
        if (NAO) {
            src = naoVision.getImageFrom(NaoVision::BOTTOM_CAMERA);
        } else {
            cap >> src;
            naoVision.setSourceMat(src);
        }

        blackArea = naoVision.getAreaBlackColor(src);

        if(finalArea > 85 || yellowArea >= 80 )
           cout<< "Black Area : "<< blackArea << endl;
            //finish = true;
        else{
            if(finalArea > 50 && finalArea < 75)
                naoMovement.middleZoneRelayRace();
            else{
                //angleToBlackLine = naoVision.calculateAngleToBlackLine();
                //naoMovement.moveInIndividualRace(angleToBlackLine);
            }
         }

        cout << "Final Area  : " << finalArea << endl;
        cout << "FlagGoalIsNear = " << flagGoalIsNear << endl;
        cout << "-----------------------------------------"<<endl;

        key = waitKey(10);
        for (int i = 0; i < 250000; i++);   // Delay.

    }

    naoVision.unsubscribe();
    naoMovement.stop();

*/
