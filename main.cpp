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

AL::ALValue walk();

int main(int argc, char *argv[]) {
    const int port = 9559;
    string ip = argv[1];        // NAO ip
    cout << "IP: " << ip << endl;

    //AL::ALRobotPostureProxy posture(ip, port);  // Posture Proxy
    //AL::ALMotionProxy motion(ip, port);         // Motion Proxy
    Mat src;
    Mat imgOriginal;
    bool DEBUG = true;          // Bandera para mostrar mensajes
    bool LOCAL = false;         // Bandera para el tipo de ejecucion (local o remota)
    bool NAO = true;
    char key = 'x';
    double angleToBlackLine;         // Angulo de la línea detectada

    NaoVision naoVision(ip, port, LOCAL);
    NaoMovement naoMovement(ip, port, LOCAL);
    VideoCapture cap(1);        // Class for video capturing from video files or cameras.

    bool start = false;
    naoMovement.initialPositionRelay();

    while (!start) { //esperando a la activacion de mi funcion
        imgOriginal = naoVision.getImageTop();
        start = naoVision.filtroColor(imgOriginal);

        key = waitKey(10);
    }

    naoVision.unsubscribe();
    naoMovement.stop();

    /*
    //Inicializacion normal
    while (key != 27) {
        if (NAO) {
            src = naoVision.getImage();
        } else {
            cap >> src;
            naoVision.setSourceMat(src);
        }

        angleToBlackLine = naoVision.calculateAngleToBlackLine();
        naoMovement.moveInIndividualRace(angleToBlackLine);
        key = waitKey(10);

        for (int i = 0; i < 250000; i++);   // Delay.
    }

    naoVision.unsubscribe();
    naoMovement.stop();
    */
    return 0;
}
